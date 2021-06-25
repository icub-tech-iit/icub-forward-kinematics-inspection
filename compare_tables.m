
DHsource = readtable('resources/icubarm_source_dhparams.csv');
DHsource.Properties.VariableNames = {'a', 'd', 'alpha', 'offset', 'min', 'max', 'n'};
RobotSource = Revolute('d', DHsource.d(1), 'a', DHsource.a(1), 'alpha', deg2rad(DHsource.alpha(1)), 'offset', deg2rad(DHsource.offset(1)));
for i=2:height(DHsource)
    RobotSource = RobotSource + Revolute('d', DHsource.d(i), 'a', DHsource.a(i), 'alpha', deg2rad(DHsource.alpha(i)), 'offset', deg2rad(DHsource.offset(i)));
end
RobotSource.name = 'iKin';
RobotSource.base = [0	-1	0	0;
                    0	0	-1	0;
                    1	0	0	0;
                    0	0	0	1];
RobotSource.tool = eye(4);

DHidyn = readtable('resources/icubgenova02_urdf_dhparams.csv');
DHidyn.Properties.VariableNames = {'a', 'd', 'alpha', 'offset', 'min', 'max'};
RobotIDyn = Revolute('d', DHidyn.d(1), 'a', DHidyn.a(1), 'alpha', deg2rad(DHidyn.alpha(1)), 'offset', deg2rad(DHidyn.offset(1)));
for i=2:height(DHsource)
   RobotIDyn = RobotIDyn + Revolute('d', DHidyn.d(i), 'a', DHidyn.a(i), 'alpha', deg2rad(DHidyn.alpha(i)), 'offset', deg2rad(DHidyn.offset(i)));
end
RobotIDyn.name = 'iDynTree';

homtable = readtable('resources/icubgenova02_urdf_h0_hn.txt');
h = zeros(4,4);
k = 2;
for i=1:4
    for j=1:4
        h(i, j) = homtable{1, k};
        k = k+1;
    end
end
RobotIDyn.base = h;
                
k = 2;
for i=1:4
    for j=1:4
        h(i, j) = homtable{2, k};
        k = k+1;
    end
end
RobotIDyn.tool = h;

qdes1 = [0 0 0 0 0 0 60.5 0 0 0];
qdes2 = [0 0 0 0 0 0 90.5 0 0 0];
qdes3 = [0 0 0 0 135 0 90.5 0 0 0];
qdes4 = [0 0 0 0 135 0 90.5 -90 -30.6 20.4];
qdes5 = [25 0 -9.6, 9.45 160.80 79.56 19.005 0 0 0.6];
qdes6 = [25 10 -9.6, 9.45 160.80 79.56 19.005 0 0 0.6];
qdes7 = [25 -15 -9.6, 9.45 160.80 79.56 19.005 0 0 5];
qdes8 = [-25 30 -9.6, 9.45 160.80 79.56 19.005 0 10 5];

Q = deg2rad(qdes2);
RobotIDyn.display
RobotIDyn.fkine(Q)

RobotSource.display
RobotSource.fkine(Q)

%% Plot Arms

hold on
RobotSource.plot(Q, 'jointcolor', 'b', 'linkcolor', 'r', 'jointdiam', 0.5, ...
    'nojoints', 'workspace', [-1 1 -1 1 -1 1], ...
    'noshading', 'noname', 'noshadow');
    zlim([-1, 1]);

alpha(.05)
hold on
RobotIDyn.plot(Q, 'jointcolor', 'r', 'linkcolor', 'b', 'jointdiam', 0.5,...
    'nojoints', 'workspace', [-1 1 -1 1 -1 1], ...
    'noshading',  'noname', 'noshadow', 'nobase');
    zlim([-0.5, 0.6]);
alpha(.05)

annotation('textbox', [.5 .5 .3 .3], 'String', 'red: iKin   blue: iDynTree','FitBoxToText','on');


%% Test links transforms
ntrials = 4;
Q = zeros(ntrials, 10);
Q(1, :) = deg2rad(qdes5);
Q(2, :) = deg2rad(qdes6);
Q(3, :) = deg2rad(qdes7);
Q(4, :) = deg2rad(qdes8);

e_qdes1 = zeros(10, 3, ntrials);

for k=1:ntrials
[iKinHee, iKinTransforms] = RobotSource.fkine(Q(k, :));
[iDynTreeHee, iDynTreeTransforms] = RobotIDyn.fkine(Q(k, :));
    for i=1:10
        e_qdes1(i, :, k) = iDynTreeTransforms(i).transl - iKinTransforms(i).transl;
    end
end 
figure('renderer', 'painters')
subplot(3,1,1)
hold on
grid minor
for i=1:ntrials
    e = e_qdes1(:, 1, i);
    plot(e * 1000, 'o-', 'LineWidth', 1.2);
end
legend({'test5', 'test6', 'test7', 'test8', 'test5'});
title('Translational error between iDynTree and iKin with four poses');
ylabel('error X [mm]');

subplot(3,1,2)
hold on
grid minor
for i=1:ntrials
    e = e_qdes1(:, 2, i);
plot(e * 1000, 'o-', 'LineWidth', 1.2);
end
ylabel('error Y [mm]');

subplot(3,1,3)
hold on
grid minor
for i=1:ntrials
    e = e_qdes1(:, 3, i);
    plot(e * 1000, 'o-', 'LineWidth', 1.2);
end
ylabel('error Z [mm]');
xlabel('reference frame index');

%% Verify orientation error for torso joints
ntrials = 4;
Q = zeros(ntrials, 10);
Q(1, :) = deg2rad(qdes5);
Q(2, :) = deg2rad(qdes6);
Q(3, :) = deg2rad(qdes7);
Q(4, :) = deg2rad(qdes8);

torsojointsnum = 10;

error_orient = zeros(torsojointsnum, 3, ntrials);

for k=1:ntrials
[iKinHee, iKinTransforms] = RobotSource.fkine(Q(k, :));
[iDynTreeHee, iDynTreeTransforms] = RobotIDyn.fkine(Q(k, :));
    
    for i=1:torsojointsnum
        Hd = iDynTreeTransforms(i).double;
        Rd = Hd(1:3, 1:3);
        He = iKinTransforms(i).double;
        Re = He(1:3, 1:3);
        R = Rd * Re';
        axang = rotm2axang(R);
        r = axang(1:3);
        theta = axang(4);
        error_orient(i, :, k) = r * sin(theta);        
    end
end

figure('renderer', 'painters')
grid minor
hold on
for i=1:ntrials
    error_norm = vecnorm(error_orient(:, :, i), 2, 2);
    plot(error_norm, 'o-', 'LineWidth', 1.2);
end
legend({'test5', 'test6', 'test7', 'test8'});
title('Norm of orientation error between iDynTree and iKin with four poses - aixs angle computation')
xlabel('reference frame index')
ylabel('norm')

figure('renderer', 'painters')
subplot(3,1,1)
hold on
grid minor
for i=1:ntrials
    plot(error_orient(:, 1, i), 'o-', 'LineWidth', 1.2);
end
legend({'test5', 'test6', 'test7', 'test8'});
title('Orientation error between iDynTree and iKin with four poses');
ylabel('n ');

subplot(3,1,2)
hold on
grid minor
for i=1:ntrials
    plot(error_orient(:, 2, i), 'o-', 'LineWidth', 1.2);
end
ylabel('s');

subplot(3,1,3)
hold on
grid minor
for i=1:ntrials
    plot(error_orient(:, 3, i), 'o-', 'LineWidth', 1.2);
end
ylabel('a');
xlabel('reference frame index');


%% Test  500 random configurations

njoints = 10;
ntrials = 500;
std = 60;
qrand = fix(std.*rand(ntrials, njoints));

IdynTransl = zeros(ntrials, 3);
SourceTransl = zeros(ntrials, 3);
for i=1:ntrials
    H = RobotIDyn.fkine(qrand(i, :)).double;
    IdynTransl(i, :) = H(1:3, 4);
    H = RobotSource.fkine(qrand(i, :)).double;
    SourceTransl(i, :) = H(1:3, 4) ;
end

e_trials = IdynTransl - SourceTransl;
subplot(3,1,1)
histogram(e_trials(:,1) * 1000, 50)
subplot(3,1,2)
histogram(e_trials(:,2) * 1000, 50)
subplot(3,1,3)
histogram(e_trials(:,3) * 1000, 50)
grid minor
legend({'e_x','e_y','e_z'})
ylabel('n. occurrences')
xlabel('Error [mm]')
%% Test handpicked configs

anglesdeg = repmat([-90, -60, -30, -15, 0, 15, 30, 60, 90]', 1, njoints);
