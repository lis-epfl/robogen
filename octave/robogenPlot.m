%% Octave-compatible script to plot robot trajectory and obstacles
% TODO test with Matlab
clear all;
close all;

%% Plot trajectory
figure();
traj = load("trajectoryLog.txt");
plot(traj(:,1),traj(:,2));

%% Plot obstacles
try
	hold on;
	obs = load("obstacles.txt");
	for i=1:rows(obs)
		o_r = obs(i,:);
		o_x = [o_r(1)-o_r(3)/2, o_r(1)+o_r(3)/2, o_r(1)+o_r(3)/2, o_r(1)-o_r(3)/2, ...
						o_r(1)-o_r(3)./2];
		o_y = [o_r(2)-o_r(4)/2,	o_r(2)-o_r(4)/2, o_r(2)+o_r(4)/2,	o_r(2)+o_r(4)/2, ...
						o_r(2)-o_r(4)/2];
		% because the ubuntu-default octave won't support the rectangle() function
		plot(o_x,o_y);
	end
	hold off;
catch
	disp "Empty or non-standard-named obstacle text file. Can't load."
end_try_catch
axis("square");
title("Robot trajectory");

%% Plot sensor values
sens = load("sensorLog.txt");
labelFile = fopen("sensorLabels.txt");
for i=1:columns(sens)
	figure();
	plot(1:rows(sens), sens(:,i));
	title(fgetl(labelFile));
end

%% Plot motor values
figure();
mot = load("motorLog.txt");
plot(1:rows(mot), mot);
axis([0 rows(mot) -0.1 1.1]);
title("Motor history");
