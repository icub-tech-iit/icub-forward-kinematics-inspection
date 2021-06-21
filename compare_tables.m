DHsource = readtable('icubarm_source_dhparams.csv');
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

DHidyn = readtable('dhparams_2.5.csv');
DHidyn.Properties.VariableNames = {'a', 'd', 'alpha', 'offset', 'min', 'max'};
RobotIDyn = Revolute('d', DHidyn.d(1), 'a', DHidyn.a(1), 'alpha', DHidyn.alpha(1), 'offset', DHidyn.offset(1));
for i=2:height(DHsource)
    RobotIDyn = RobotIDyn + Revolute('d', DHidyn.d(i), 'a', DHidyn.a(i), 'alpha', DHidyn.alpha(i), 'offset', DHidyn.offset(i));
end
RobotIDyn.name = 'iDynTree';
RobotIDyn.base = [1	0	0	0;
                    0	0	-1	0.0269348;
                    0	1	0	0;
                    0	0	0	1];
RobotIDyn.tool = [1	0.0286	0	0;
                    -0.0286	1	0	0;
                    0	0	1	0;
                    0	0	0	1];
q0 = zeros(length(RobotIDyn.links));

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