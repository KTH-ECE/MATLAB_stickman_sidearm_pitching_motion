clear; clc; close all;

%% ===================== SETTINGS =====================
videoFile     = "pitching.mp4";
resizeFactor  = 0.5;
saveFile      = "tracked_points.mat";
displayEvery  = 1;        % render 1 of every N frames (tracking still runs every frame)
preloadFrames = true;     % set false if RAM is tight

jointNames = [ ...
    "ankle1","ankle2", ...
    "knee1","knee2", ...
    "hip1","hip2", ...
    "shoulder1","shoulder2", ...
    "elbow1","elbow2", ...
    "wrist1","wrist2", ...
    "head"];
numJoints = numel(jointNames);

%% ===================== VIDEO INFO =====================
v = VideoReader(videoFile);
fps       = v.FrameRate;
numFrames = floor(v.Duration * v.FrameRate);

%% ===================== LOAD / INIT STATE =====================
if isfile(saveFile)
    S = load(saveFile);
    points    = S.points;
    startFrame = S.lastFrame + 1;
    initialPoints = squeeze(points(S.lastFrame, :, :));   % last known good positions

    if startFrame > numFrames
        fprintf("Already finished. Delete %s to start over.\n", saveFile);
        return;
    end
    fprintf("Resuming from frame %d / %d\n", startFrame, numFrames);
    needInitialClicks = false;
else
    points = nan(numFrames, numJoints, 2);
    startFrame = 2;
    needInitialClicks = true;
    fprintf("No previous save. Starting fresh.\n");
end

%% ===================== PRELOAD FRAMES =====================
if preloadFrames
    fprintf("Preloading frames... ");
    tic;
    allFrames = cell(numFrames, 1);
    v.CurrentTime = 0;
    i = 1;
    while hasFrame(v) && i <= numFrames
        allFrames{i} = imresize(readFrame(v), resizeFactor);
        i = i + 1;
    end
    numFrames = i - 1;        % adjust if duration*fps was off
    fprintf("done (%.1fs, %d frames)\n", toc, numFrames);
    getFrame = @(idx) allFrames{idx};
else
    getFrame = @(idx) readFrameAt(v, idx, resizeFactor);
end

%% ===================== INITIAL CLICKS (FRESH RUN ONLY) =====================
fig = figure('Name','Joint Tracker','NumberTitle','off');

if needInitialClicks
    frame = getFrame(1);
    imshow(frame); hold on;
    title("Click all 13 joints in order shown in console");

    initialPoints = zeros(numJoints, 2);
    for j = 1:numJoints
        fprintf("Click: %s\n", jointNames(j));
        [x, y] = ginput(1);
        initialPoints(j, :) = [x, y];
        points(1, j, :) = [x, y];
        plot(x, y, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
        text(x+5, y, jointNames(j), 'Color','yellow','FontSize',8);
    end
    drawnow;
    pause(0.5);
end

%% ===================== TRACKER SETUP =====================
tracker = vision.PointTracker( ...
    'MaxBidirectionalError', 3, ...
    'NumPyramidLevels', 3, ...
    'BlockSize', [21 21]);

frame0    = getFrame(startFrame - 1);
frameGray = rgb2gray(frame0);
initialize(tracker, initialPoints, frameGray);

%% ===================== PERSISTENT DISPLAY HANDLES =====================
clf(fig);
hImg = imshow(getFrame(startFrame - 1)); hold on;
hPts   = gobjects(numJoints, 1);
hLabel = gobjects(numJoints, 1);
for j = 1:numJoints
    hPts(j)   = plot(NaN, NaN, 'go', 'MarkerSize', 8, 'LineWidth', 2);
    hLabel(j) = text(NaN, NaN, jointNames(j), 'Color','yellow','FontSize',7);
end
hTitle = title("");
set(fig, 'CurrentCharacter', ' ');

fprintf("\n>>> Tracking. Press 'p' to pause & fix, 'q' to quit & save. <<<\n\n");

%% ===================== MAIN TRACKING LOOP =====================
quitRequested = false;

for f = startFrame:numFrames

    frame     = getFrame(f);
    frameGray = rgb2gray(frame);

    [trackedPoints, validity] = tracker(frameGray);

    %% Save tracked points
    for j = 1:numJoints
        if validity(j)
            points(f, j, :) = trackedPoints(j, :);
        else
            points(f, j, :) = [NaN, NaN];
        end
    end

    %% Throttled display
    if mod(f, displayEvery) == 0 || f == numFrames
        set(hImg, 'CData', frame);
        for j = 1:numJoints
            if validity(j)
                set(hPts(j), 'XData', trackedPoints(j,1), 'YData', trackedPoints(j,2), ...
                    'Marker','o','Color','g');
                set(hLabel(j), 'Position', [trackedPoints(j,1)+5, trackedPoints(j,2), 0]);
            else
                set(hPts(j), 'Marker','x','Color','r');
            end
        end
        set(hTitle, 'String', sprintf('Frame %d / %d   (p=fix, q=quit)', f, numFrames));
        drawnow;
        pause(0.03);
    end

    %% Check key press
    key = get(fig, 'CurrentCharacter');

    if key == 'p'
        set(fig, 'CurrentCharacter', ' ');

        % Force a full redraw so user sees current frame clearly
        set(hImg, 'CData', frame);
        for j = 1:numJoints
            if validity(j)
                set(hPts(j), 'XData', trackedPoints(j,1), 'YData', trackedPoints(j,2));
            end
        end
        drawnow;

        fprintf("\n--- PAUSED at frame %d ---\n", f);
        fprintf("For each joint: click new position, OR press ENTER to keep current.\n");

        correctedPoints = trackedPoints;   % start with current
        needsReinit = false;

        for j = 1:numJoints
            fprintf("  %s (ENTER to skip): ", jointNames(j));

            % Highlight which joint we're fixing
            set(hPts(j), 'Color','m','MarkerSize',14);
            drawnow;

            [xFix, yFix, btn] = ginput(1);

            if isempty(btn)   % user pressed ENTER
                fprintf("kept\n");
                set(hPts(j), 'Color','g','MarkerSize',8);
            else
                correctedPoints(j, :) = [xFix, yFix];
                points(f, j, :) = [xFix, yFix];
                set(hPts(j), 'XData', xFix, 'YData', yFix, 'Color','b','MarkerSize',8);
                fprintf("fixed -> (%.0f, %.0f)\n", xFix, yFix);
                needsReinit = true;
            end
        end

        if needsReinit
            release(tracker);
            tracker = vision.PointTracker( ...
                'MaxBidirectionalError', 3, ...
                'NumPyramidLevels', 3, ...
                'BlockSize', [21 21]);
            initialize(tracker, correctedPoints, frameGray);
            fprintf("Tracker re-initialised. Continuing.\n\n");
        else
            fprintf("No changes. Continuing.\n\n");
        end

        % Save progress after every fix
        lastFrame = f;
        save(saveFile, 'points','jointNames','fps','resizeFactor','lastFrame');

    elseif key == 'q'
        set(fig, 'CurrentCharacter', ' ');
        fprintf("\nQuit requested at frame %d. Saving...\n", f);
        quitRequested = true;
        break;
    end

    %% Periodic autosave (every ~5 seconds of video)
    if mod(f, round(5*fps)) == 0
        lastFrame = f;
        save(saveFile, 'points','jointNames','fps','resizeFactor','lastFrame');
    end
end

%% ===================== FINAL SAVE =====================
release(tracker);

if quitRequested
    lastFrame = f;
else
    lastFrame = numFrames;
end
save(saveFile, 'points','jointNames','fps','resizeFactor','lastFrame');

if quitRequested
    fprintf("Saved progress up to frame %d. Re-run script to resume.\n", lastFrame);
else
    fprintf("Done! All %d frames tracked. Saved to %s\n", numFrames, saveFile);
end

%% ===================== HELPER (only used if preloadFrames=false) =====================
function f = readFrameAt(v, idx, rf)
    v.CurrentTime = (idx - 1) / v.FrameRate;
    f = imresize(readFrame(v), rf);
end