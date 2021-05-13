global FONT
FONT = 24;
filelist = dir;
csvlist = [];
for i = 1:size(filelist,1)
    if contains(filelist(i).name, 'csv')
        csvlist = [csvlist; string(filelist(i).name)];
    end
end

Robot_files = containers.Map('KeyType','uint32','ValueType','any');
pos_files = [];
robot_count = 0;
for i = 1:size(csvlist, 1)
    if contains(csvlist(i), 'pos')
        robot_count = robot_count + 1;
        pos_files = [pos_files csvlist(i)];
    end
end
for i = 0:robot_count-1
    arr = [];
    for j = 1:size(csvlist, 1)
        char_arr = char(csvlist(j));
        char1 = char_arr(1);
        char6 = char_arr(6);
        if (contains(csvlist(j), 'comp') & (char1 == string(i)))
            arr = [arr csvlist(j)];
        elseif (contains(csvlist(j), 'robot') & (char6 == string(i)))
            arr = [arr csvlist(j)];
        else
            continue
        end
    end
    Robot_files(i) = arr;
end
disp('ok')
for i = 1:size(Robot_files,1)-1
    get_vel_plot(Robot_files(i));
    get_err_plot(Robot_files(i));
    get_comp_plot(Robot_files(i), 'xi');
    get_comp_plot(Robot_files(i), 'dxi');
    get_comp_plot(Robot_files(i), 'k');
    get_comp_plot(Robot_files(i), 'dk');
    get_comp_plot(Robot_files(i), 'u');
end
get_pos_plot(pos_files);

function get_vel_plot(filelist)
global FONT
    for i = 1:size(filelist, 2)
       if contains(filelist(i), 'vel')
            arr = csvread(filelist(i), 1);
            filename = char(filelist(i));
            filename = filename(1:end-4);
            path = 'graph/' + string(filename) + '.png';
            figpath = 'graph/figs/' + string(filename) + '.fig';
            t = arr(:,1) - arr(1,1);
            x = arr(:,2);
            y = arr(:,3);
            h = figure('Visible','off', 'Position', [0 0 1368 720], 'DefaultAxesFontSize', FONT);
            grid on;
            grid minor;
            hold on;
            plot(t,x, 'DisplayName','v_x');
            plot(t,y, 'DisplayName','v_y');
            hold off;
            xlabel('t, c', 'FontSize', FONT)
            ylabel('v, м/с^2', 'FontSize', FONT)
            legend;
            xlim([0 max(t)])
            ylim([min([x; y])-abs(min([x; y]))*0.1 max([x; y]) + abs(max([x; y]))*0.1]);
            print(h,'-dpng', path);
            savefig(h, figpath);
            close(h);
       else
            continue
       end
    end
end

function get_err_plot(filelist)
global FONT
filename = char(filelist(1));
i = filename(1);
filename = 'robot' + string(i) + '_errors';
path = 'graph/' + string(filename) + '.png';
figpath = 'graph/figs/' + string(filename) + '.fig';
h = figure('Visible','off', 'Position', [0 0 1368 720], 'DefaultAxesFontSize', FONT);
hold on;
xlabel('t, c')
ylabel('Ошибка, м')    
grid on;
grid minor
x_min = 0;
x_max = 0;
for i = 1:size(filelist, 2)
    if contains(filelist(i), 'err')
        filename = char(filelist(i));
        legend_name = 'robot_' + string(filename(11)) + 'error';
        arr = csvread(filelist(i), 1);
        t = arr(:,1) - arr(1,1);
        x = arr(:,2);
        plot(t,x, 'DisplayName',string(legend_name));
        if max(x) > x_max
            x_max = max(x)+abs(max(x))*0.1;
        end
        if min(x) < x_min
            x_min = min(x)-abs(min(x))*0.1;
        end      
        ylim([x_min, x_max])
        xlim([0, max(t)])
    else
        continue
    end
end
legend
      
print(h,'-dpng', path);
savefig(h, figpath);
close(h);
end

function get_pos_plot(filelist)
global FONT
path = 'graph/robot_positions.png';
figpath = 'graph/figs/robot_positions.fig';
h = figure('Visible','off', 'Position', [0 0 1368 720], 'DefaultAxesFontSize', FONT);
set(h, 'DefaultTextFontSize', 38);
hold on;
xlabel('x')
ylabel('y')    
grid on;
grid minor;
for i = 1:size(filelist, 2)
        filename = char(filelist(i));
        legend_name = 'robot_' + string(filename(6));
        arr = csvread(filelist(i), 1);
        x = arr(:,2);
        y = arr(:,3);
        plot(x,y, 'DisplayName',string(legend_name), 'LineWidth', 3);
end
legend
print(h,'-dpng', path);
savefig(h, figpath);
close(h);
end

function get_comp_plot(filelist, var)
global FONT
filename = char(filelist(1));
path = 'graph/comp' + string(filename(1)) + '_' + var + '.png';
figpath = 'graph/figs/' + string(filename) + '_' + var + '.fig';
h = figure('Visible','off', 'Position', [0 0 1368 720], 'DefaultAxesFontSize', FONT);
if string(var) == 'dk'
    x = 2;
    ylabel('$\dot{k}$', 'Interpreter','latex')
elseif string(var) == 'k'
    x = 3;
    ylabel('${k}$', 'Interpreter','latex')
elseif string(var) == 'dxi'
    x = 4;
    ylabel('$\dot{\xi}$', 'Interpreter','latex')
elseif string(var) == 'xi'
    x = 5;
    ylabel('${\xi}$', 'Interpreter','latex')
elseif string(var) == 'u'
    x = 6;
    ylabel('${u}$', 'Interpreter','latex')
end
hold on;
xlabel('t, c')  
grid on;
grid minor
y_min = 0;
y_max = 0;
for i = 1:size(filelist, 2)
    if contains(filelist(i), 'comp')
        filename = char(filelist(i));
        filename = replace(filename,'_','-');
        legend_name = 'comp ' + string(filename(1:6)) + ' ' + var;
        arr = csvread(filelist(i), 1);
        t = arr(:,1) - arr(1,1);
        y = arr(:,x);
              
        if max(y) > y_max
            y_max = max(y)+abs(max(y))*0.1;
        end
        if min(y) < y_min
            y_min = min(y)-abs(min(y))*0.1;
        end      
        ylim([y_min, y_max])
        xlim([0, max(t)])
        plot(t,y, 'DisplayName',string(legend_name));
    else
        continue
    end
end
legend
print(h,'-dpng', path);
savefig(h, figpath);
close(h);
end

