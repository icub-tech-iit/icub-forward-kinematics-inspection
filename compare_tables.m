DHsource = readtable('resources/icubarm_source_dhparams.csv');
DHsource.Properties.VariableNames = {'a', 'd', 'alpha', 'offset', 'min', 'max'};
RobotSource = Revolute('d', DHsource.d(1), 'a', DHsource.a(1), 'alpha', deg2rad(DHsource.alpha(1)), 'offset', deg2rad(DHsource.offset(1)));
for i=2:height(DHsource)
    RobotSource = RobotSource + Revolute('d', DHsource.d(i), 'a', DHsource.a(i), 'alpha', deg2rad(DHsource.alpha(i)), 'offset', deg2rad(DHsource.offset(i)));
end
RobotSource.name = 'iKin';
RobotSource.base = [0	-1	0	0;
                    0	0	-1	0;
                    1	0	0	0;
                    0	0	0	1];
RobotSource.tool = [1	0	0	0;
                    0	1	0	0;
                    0	0	1	0;
                    0	0	0	1];

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

% sourceTransform = RobotSource.base.double;
% q = q0 + deg2rad(RobotSource.offset);
% for i=1:10
%     ai = RobotSource.a(i);
%     di = RobotSource.d(i);
%     alphai = deg2rad(RobotSource.alpha(i));
%     qi = q(i);
%     Ti = [cos(qi), -sin(qi)*cos(alphai), sin(qi)*sin(alphai), ai*cos(qi);
%          sin(qi), cos(qi)*cos(alphai), -cos(qi)*sin(alphai), ai*sin(qi);
%          0, sin(alphai), cos(alphai), di;
%          0, 0, 0, 1];
%     
%    sourceTransform = sourceTransform * Ti;
% end
% 
% sourceTransform = sourceTransform * RobotSource.tool.double;

qdes1 = [0 0 0 0 0 0 60.5 0 0 0];
qdes2 = [0 0 0 0 0 0 90.5 0 0 0];
qdes3 = [0 0 0 0 90 -30 15 0 0 0];
qdes4 = [0 0 0 0 135 0 90.5 -90 -30.6 20.4];

Q = deg2rad(qdes2);

hold on
RobotSource.plot(Q, 'jointcolor', 'b', 'linkcolor', 'r', 'jointdiam', 0.5, ...
    'nojoints', 'workspace', [-1 1 -1 1 -1 1], ...
    'noshading', 'noname', 'noshadow');
    zlim([-1, 1]);

alpha(.5)
hold on
RobotIDyn.plot(Q, 'jointcolor', 'r', 'linkcolor', 'b', 'jointdiam', 0.5,...
    'nojoints', 'workspace', [-1 1 -1 1 -1 1], ...
    'noshading',  'noname', 'noshadow');
    zlim([-1, 1]);

annotation('textbox', [.5 .5 .3 .3], 'String', 'red: iKin   blue: iDynTree','FitBoxToText','on');
exportgraphics(gcf(), 'zeros.png');


RobotIDyn.display
RobotIDyn.fkine(Q)
sprintf('IDyn H0')
RobotIDyn.base
sprintf('IDyn EE')
RobotIDyn.tool

RobotSource.display
RobotSource.fkine(Q)
sprintf('IDyn H0')
RobotSource.base
sprintf('IDyn EE')
RobotSource.tool

model = importrobot('model/model.urdf');
fromtorso = subtree(model, 'root_link');