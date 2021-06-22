DHsource = readtable('resources/icubarm_source_dhparams.csv');
DHsource.Properties.VariableNames = {'a', 'd', 'alpha', 'offset', 'min', 'max'};
RobotSource = Revolute('d', DHsource.d(1), 'a', DHsource.a(1), 'alpha', DHsource.alpha(1), 'offset', DHsource.offset(1));
for i=2:height(DHsource)
    RobotSource = RobotSource + Revolute('d', DHsource.d(i), 'a', DHsource.a(i), 'alpha', DHsource.alpha(i), 'offset', DHsource.offset(i));
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
for i=2:height(DHidyn)
    RobotIDyn = RobotIDyn + Revolute('d', DHidyn.d(i), 'a', DHidyn.a(i), 'alpha', deg2rad(DHidyn.alpha(i)), 'offset', deg2rad(DHidyn.offset(i)));
end
RobotIDyn.name = 'iDynTree';

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

q0 = [0.0 90.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 90.0];

q0 = zeros(1, 10);

iDynTransform = RobotSource.base.double;
q = q0 + deg2rad(RobotSource.offset);
for i=1:10
    ai = RobotSource.a(i);
    di = RobotSource.d(i);
    alphai = deg2rad(RobotSource.alpha(i));
    qi = q(i);
    Ti = [cos(qi), -sin(qi)*cos(alphai), sin(qi)*sin(alphai), ai*cos(qi);
         sin(qi), cos(qi)*cos(alphai), -cos(qi)*sin(alphai), ai*sin(qi);
         0, sin(alphai), cos(alphai), di;
         0, 0, 0, 1];
    
   iDynTransform = iDynTransform * Ti;
end

iDynTransform = iDynTransform * RobotSource.tool.double;

hold on

RobotSource.plot(q0, 'jointcolor', 'b', 'linkcolor', 'r', 'jointdiam', 0.5, 'nojoints', 'workspace', [-1 1 -1 1 -1 1]);

hold on

RobotIDyn.plot(q0, 'jointcolor', 'c', 'linkcolor', 'g', 'jointdiam', 0.5,  'nojoints', 'workspace', [-1 1 -1 1 -1 1]);
annotation('textbox', [.5 .5 .3 .3], 'String', 'red: iKin   green: iDynTree','FitBoxToText','on');
exportgraphics(gcf(), 'zeros.png');

RobotIDyn.display
sprintf('IDyn H0')
RobotIDyn.base
sprintf('IDyn EE')
RobotIDyn.tool

RobotSource.display
sprintf('IDyn H0')
RobotSource.base
sprintf('IDyn EE')
RobotSource.tool