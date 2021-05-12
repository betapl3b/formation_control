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
end
get_pos_plot(pos_files);

function get_vel_plot(filelist)
    for i = 1:size(filelist, 2)
       if contains(filelist(i), 'vel')
            arr = csvread(filelist(i), 1);
            filename = char(filelist(i));
            filename = filename(1:end-4);
            path = 'graph/' + string(filename) + '.png';
            t = arr(:,1) - arr(1,1);
            x = arr(:,2);
            y = arr(:,3);
            h = figure('Visible','off');
            hold on;
            plot(t,x, 'DisplayName','v_x');
            plot(t,y, 'DisplayName','v_y');
            hold off;
            xlabel('t, c')
            ylabel('v, м/с^2')
            legend;
            print(h,'-dpng', path);
            close(h);
       else
            continue
       end
    end
end

function get_err_plot(filelist)
filename = char(filelist(1));
i = filename(1);
filename = 'robot' + string(i) + '_errors';
path = 'graph/' + string(filename) + '.png';
figpath = 'graph/' + string(filename) + '.fig';
h = figure('Visible','off');
hold on;
xlabel('t, c')
ylabel('Ошибка, м')    
grid on;
for i = 1:size(filelist, 2)
    if contains(filelist(i), 'err')
        filename = char(filelist(i));
        legend_name = 'robot_' + string(filename(11)) + 'error';
        arr = csvread(filelist(i), 1);
        t = arr(:,1) - arr(1,1);
        x = arr(:,2);
        y = arr(:,3);
        plot(t,x, 'DisplayName',string(legend_name) + '_x');
        plot(t,y, 'DisplayName',string(legend_name) + '_y');
    else
        continue
    end
end
legend
print(h,'-dpng', path);
saveas(h, figpath, 'fig');
close(h);
end

function get_pos_plot(filelist)
path = 'graph/robot_positions.png';
h = figure('Visible','off');
hold on;
xlabel('x')
ylabel('y')    
grid on;
for i = 1:size(filelist, 2)
        filename = char(filelist(i));
        legend_name = 'robot_' + string(filename(6));
        arr = csvread(filelist(i), 1);
        x = arr(:,2);
        y = arr(:,3);
        plot(x,y, 'DisplayName',string(legend_name));
end
legend
print(h,'-dpng', path);
close(h);
end
