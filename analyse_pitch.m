clear; clc; close all;

%% =========================
%  SETTINGS
% =========================
videoFile   = "pitching.mp4";
trackedFile = "tracked_points.mat";

% Smoothing
smoothWindow = 7;        % must be odd
smoothOrder  = 3;

% Output
makeVideo    = true;
videoOutFile = "stickman_pitch.avi";

%% =========================
%  LOAD VIDEO + TRACKED DATA
% =========================
v = VideoReader(videoFile);
fps = v.FrameRate;
dt  = 1 / fps;

S = load(trackedFile);
points       = S.points;
jointNames   = S.jointNames;
resizeFactor = S.resizeFactor;
numFrames    = size(points, 1);
numJoints    = numel(jointNames);

fprintf("Loaded %d frames, %d joints, %.2f fps\n", numFrames, numJoints, fps);

% Un-resize back to original video coordinates
points_orig = points / resizeFactor;

%% =========================
%  SMOOTH TRAJECTORIES
% =========================
points_smooth = points_orig;

for j = 1:numJoints
    for d = 1:2
        traj = points_orig(:, j, d);
        validIdx = find(~isnan(traj));
        if numel(validIdx) < smoothWindow
            continue;
        end
        traj_filled = interp1(validIdx, traj(validIdx), (1:numFrames)', ...
                              'linear', 'extrap');
        points_smooth(:, j, d) = sgolayfilt(traj_filled, smoothOrder, smoothWindow);
    end
end

%% =========================
%  FLIP Y (image down -> world up) and shift to positive
% =========================
points_plot = points_smooth;
points_plot(:, :, 2) = -points_plot(:, :, 2);
points_plot(:, :, 2) = points_plot(:, :, 2) - min(points_plot(:, :, 2), [], 'all');

%% =========================
%  SAVE
% =========================
save("pitch_kinematics.mat", ...
    "points_smooth", "points_plot", "jointNames", "fps", "dt");
fprintf("Saved pitch_kinematics.mat\n");

%% =========================
%  STICKMAN VIDEO
% =========================
if ~makeVideo
    return;
end

connections = [
    1 3;  3 5;             % left leg
    2 4;  4 6;             % right leg
    5 6;                   % hips
    5 7;  6 8;  7 8;       % torso
    7 9;  9 11;            % left arm
    8 10; 10 12;           % right arm
    7 13; 8 13];           % head

out = VideoWriter(videoOutFile);
out.FrameRate = fps;
open(out);

figVid = figure('Name','Stickman','Position',[100 100 800 800]);

xMin = min(points_plot(:,:,1), [], 'all') - 50;
xMax = max(points_plot(:,:,1), [], 'all') + 50;
yMin = min(points_plot(:,:,2), [], 'all') - 50;
yMax = max(points_plot(:,:,2), [], 'all') + 50;

ax = axes('Parent', figVid);
hold(ax, 'on');
axis(ax, 'equal');
xlim(ax, [xMin xMax]);
ylim(ax, [yMin yMax]);
xlabel(ax, 'X (px)'); ylabel(ax, 'Y (px)');
grid(ax, 'on');

hLines = gobjects(size(connections,1), 1);
for c = 1:size(connections,1)
    hLines(c) = plot(ax, NaN, NaN, 'g-', 'LineWidth', 2);
end
hJoints = plot(ax, NaN, NaN, 'ro', 'MarkerSize', 6, 'LineWidth', 2);
hTitle  = title(ax, '');

for f = 1:numFrames
    pts = squeeze(points_plot(f, :, :));

    for c = 1:size(connections,1)
        a = connections(c,1); b = connections(c,2);
        set(hLines(c), 'XData', [pts(a,1), pts(b,1)], ...
                       'YData', [pts(a,2), pts(b,2)]);
    end
    set(hJoints, 'XData', pts(:,1), 'YData', pts(:,2));
    set(hTitle, 'String', sprintf('v2 | Frame %d/%d', f, numFrames));

    drawnow limitrate;
    writeVideo(out, getframe(figVid));
end

close(out);
fprintf("Wrote %s\nDone.\n", videoOutFile);