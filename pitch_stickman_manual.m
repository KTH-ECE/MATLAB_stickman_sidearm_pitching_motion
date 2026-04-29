clear; clc; close all;

%% =========================
%  SETTINGS
% =========================

videoFile = "pitching.mp4";
bodyMass = 75;               % kg
scale_m_per_pixel = 1;

trackManually = false;        % true = click points, false = load saved data

%% =========================
%  LOAD VIDEO
% =========================

v = VideoReader(videoFile);
numFrames = floor(v.Duration * v.FrameRate);
fps = v.FrameRate;
frameStep = 77;
dt = frameStep / fps;

fprintf("Video loaded: %d frames, %.2f fps\n", numFrames, fps);

%% =========================
%  JOINT SETUP
% =========================

jointNames = [
    "ankle1","ankle2", ...
    "knee1","knee2", ...
    "hip1","hip2", ...
    "shoulder1","shoulder2", ...
    "elbow1","elbow2", ...
    "wrist1","wrist2", ...
    "head"
];

numJoints = length(jointNames);
points = nan(numFrames, numJoints, 2);

%% =========================
%  MANUAL DIGITISING
%  One joint through all frames
% =========================

if trackManually

    for j = 1:numJoints

        fprintf("\nNow tracking joint %d/%d: %s\n", ...
            j, numJoints, jointNames(j));

        pause(1);

        frameStep = 77;

        for f = 1:frameStep:numFrames

            img = read(v, f);
            imshow(img);
            hold on;

            title("Click " + jointNames(j) + ...
                  " | Frame " + f + "/" + numFrames);

            % Show already-clicked point from previous frame
            if f > 1 && ~isnan(points(f-1,j,1))
                oldX = points(f-1,j,1);
                oldY = points(f-1,j,2);
                plot(oldX, oldY, 'go', 'MarkerSize', 10, 'LineWidth', 2);
            end

            [x, y] = ginput(1);
            points(f,j,:) = [x, y];

            plot(x, y, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
            drawnow;

        end

        save("pitch_points_partial.mat", ...
            "points", "jointNames", "fps", "dt", "bodyMass");

        fprintf("Saved after joint: %s\n", jointNames(j));
    end

    save("pitch_points_final.mat", ...
        "points", "jointNames", "fps", "dt", "bodyMass");

else
    load("pitch_points_final.mat");
end

%% =========================
%  STICKMAN CONNECTIONS
% =========================

connections = [
    1 3    % ankle1 - knee1
    3 5    % knee1 - hip1

    2 4    % ankle2 - knee2
    4 6    % knee2 - hip2

    5 6    % hip1 - hip2

    5 7    % hip1 - shoulder1
    6 8    % hip2 - shoulder2
    7 8    % shoulder1 - shoulder2

    7 9    % shoulder1 - elbow1
    9 11   % elbow1 - wrist1

    8 10   % shoulder2 - elbow2
    10 12  % elbow2 - wrist2

    7 13   % shoulder1 - head
    8 13   % shoulder2 - head
];

%% =========================
%  CENTER OF MASS ESTIMATE
% =========================

COM = nan(numFrames, 2);

frameStep = 77;

for f = 1:frameStep:numFrames

    pts = squeeze(points(f,:,:));

    ankleMid    = mean(pts([1 2],:), 1, "omitnan");
    kneeMid     = mean(pts([3 4],:), 1, "omitnan");
    hipMid      = mean(pts([5 6],:), 1, "omitnan");
    shoulderMid = mean(pts([7 8],:), 1, "omitnan");
    elbowMid    = mean(pts([9 10],:), 1, "omitnan");
    wristMid    = mean(pts([11 12],:), 1, "omitnan");
    head        = pts(13,:);

    % Simple approximate body CoM model
    COM(f,:) = ...
        0.10 * ankleMid + ...
        0.15 * kneeMid + ...
        0.25 * hipMid + ...
        0.25 * shoulderMid + ...
        0.10 * elbowMid + ...
        0.05 * wristMid + ...
        0.10 * head;
end

%% =========================
%  FORCE ESTIMATE
% =========================

COM_m = COM * scale_m_per_pixel;

velocityCOM = gradient(COM_m, dt);
accelerationCOM = gradient(velocityCOM, dt);

forceCOM = bodyMass * accelerationCOM;

%% =========================
%  SAVE DATA
% =========================

save("pitch_analysis_data.mat", ...
    "points", "jointNames", "COM", "velocityCOM", ...
    "accelerationCOM", "forceCOM", "fps", "dt", "bodyMass");

%% =========================
%  CREATE STICKMAN VIDEO
% =========================

out = VideoWriter("stickman_pitch_with_CoM_force.avi");
out.FrameRate = fps;
open(out);

figure;

frameStep = 77;

for f = 1:frameStep:numFrames

    clf;
    hold on;
    axis equal;
    set(gca, 'YDir', 'reverse');

    pts = squeeze(points(f,:,:));

    % Draw stickman
    for c = 1:size(connections,1)

        a = connections(c,1);
        b = connections(c,2);

        plot([pts(a,1), pts(b,1)], ...
             [pts(a,2), pts(b,2)], ...
             'g-', 'LineWidth', 2);
    end

    % Draw joints
    plot(pts(:,1), pts(:,2), 'ro', ...
        'MarkerSize', 6, 'LineWidth', 2);

    % Draw COM
    plot(COM(f,1), COM(f,2), 'bo', ...
        'MarkerSize', 10, 'LineWidth', 3);

    text(COM(f,1)+10, COM(f,2), "COM", ...
        'Color', 'blue', 'FontSize', 12);

    % Draw force vector
    forceScale = 0.001;

    quiver(COM(f,1), COM(f,2), ...
           forceCOM(f,1) * forceScale, ...
           forceCOM(f,2) * forceScale, ...
           'r', 'LineWidth', 2, 'MaxHeadSize', 2);

    title("Frame " + f + "/" + numFrames);

    xlim([min(points(:,:,1),[],"all")-100, max(points(:,:,1),[],"all")+100]);
    ylim([min(points(:,:,2),[],"all")-100, max(points(:,:,2),[],"all")+100]);

    frame = getframe(gcf);
    writeVideo(out, frame);
end

close(out);

disp("Done.");
disp("Saved:");
disp("1. pitch_points_final.mat");
disp("2. pitch_analysis_data.mat");
disp("3. stickman_pitch_with_COM_force.avi");