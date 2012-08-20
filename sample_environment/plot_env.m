%% Load all the files

close all; clear;

load box.txt
load bot.txt
load obstacles.txt
load goal.txt


%% Draw the bounding box

% Calculate the coordinates of the bounding box
box_x = box(1); % x coordinate of the center of the box
box_y = box(2); % y coordinate of the center of the box 
box_w = box(3); % length of the box
box_l = box(4); % widhth of the box

% Itialize the figure according to the bounding box
% and draw the bounding
figure;
hold on, rectangle ('Position', [box_x-box_w/2, box_y-box_l/2, box_w, box_l]);
axis ([box_x-box_w/2-1, box_x+box_w/2+1, box_y-box_l/2-1,  box_y+box_l/2+1]);
axis equal; grid;

%% Draw the robot in blue

% Calculate the circle for the robot
bot_x = bot(1); % x coordinate of the center of the robot
bot_y = bot(2); % y coordinate of the center of the robot
bot_r = bot(3); % radius of the robot
% Represent the circle in terms of a sequence of points
[bot_cy_x bot_cy_y] = cylinder (bot_r, 100); 
bot_disk_x = bot_cy_x(1,:) + bot_x; % x coordinates of the points on the circle
bot_disk_y = bot_cy_y(1,:) + bot_y; % y coordinates of the points on the circle

% Draw the disk representing the robot
hold on, fill (bot_disk_x, bot_disk_y, 'b');


%% Draw the goal region in green

% Calculate the circle for the robot
goal_x = goal(1); % x coordinate of the center of the goal
goal_y = goal(2); % y coordinate of the center of the goal
goal_r = goal(3); % radius of the goal
% Represent the circle in terms of a sequence of points
[goal_cy_x goal_cy_y] = cylinder (goal_r, 100); 
goal_disk_x = goal_cy_x(1,:) + goal_x; % x coordinates of the points on the circle
goal_disk_y = goal_cy_y(1,:) + goal_y; % y coordinates of the points on the circle

% Draw the disk representing the robot
hold on, fill (goal_disk_x, goal_disk_y, 'g');


%% Draw all the obstacles in red

sz_obstacles = size (obstacles); 
num_obstacles = sz_obstacles (1);

for i = 1 : num_obstacles 
    obstacle_curr = obstacles (i, :);
    
    % Calculate the circle for the robot
    obstacle_x = obstacle_curr(1); % x coordinate of the center of the obstacle
    obstacle_y = obstacle_curr(2); % y coordinate of the center of the obstacle
    obstacle_r = obstacle_curr(3); % radius of the obstacle
    % Represent the circle in terms of a sequence of points
    [obstacle_cy_x obstacle_cy_y] = cylinder (obstacle_r, 100); 
    obstacle_disk_x = obstacle_cy_x(1,:) + obstacle_x; % x coordinates of the points on the circle
    obstacle_disk_y = obstacle_cy_y(1,:) + obstacle_y; % y coordinates of the points on the circle

    % Draw the disk representing the robot
    hold on, fill (obstacle_disk_x, obstacle_disk_y, 'r');
end