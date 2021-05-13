filelist = dir;
csvlist = [];
for i = 1:size(filelist,1)
    if contains(filelist(i).name, 'csv')
        csvlist = [csvlist; string(filelist(i).name)];
    end
end
for i = 1:size(csvlist, 1)
   if contains(csvlist(i), 'comp')
       if contains(csvlist(i), '0_to')
           continue
       end
       get_comp(csvlist(i));
   else
       continue
   end
end


function [t, dk, k, dxi, xi, out] = get_comp(file)
    arr = csvread(file, 1);
    filename = char(file);
    filename = filename(1:end-4);
    path = 'graph/' + string(filename) + '.png';
    t = arr(:,1) - arr(1,1);
    dk = arr(:,2);
    k = arr(:,3);
    dxi = arr(:,4);
    xi = arr(:,5);
    out = arr(:,6);
    % dxi
    path = 'graph/' + string(filename) + '_dxi.png';
    h = figure('Visible','off');
    plot(t,dxi);
    xlabel('t, c')
    ylabel('$\dot{\xi}$', 'Interpreter','latex')
    print(h,'-dpng', path);
    close(h);
    % xi
    path = 'graph/' + string(filename) + '_xi.png';
    h = figure('Visible','off');
    plot(t,xi);
    xlabel('t, c')
    ylabel('${\xi}$', 'Interpreter','latex')
    print(h,'-dpng', path);
    close(h);
    % dk
    path = 'graph/' + string(filename) + '_dk.png';
    h = figure('Visible','off');
    plot(t,dk);
    xlabel('t, c')
    ylabel('$\dot{k}$', 'Interpreter','latex')
    print(h,'-dpng', path);
    close(h);
    % k
    path = 'graph/' + string(filename) + '_k.png';
    h = figure('Visible','off');
    plot(t,k);
    xlabel('t, c')
    ylabel('k', 'Interpreter','latex')
    print(h,'-dpng', path);
    close(h);
    % out
    path = 'graph/' + string(filename) + '_u.png';
    h = figure('Visible','off');
    plot(t,out);
    xlabel('t, c')
    ylabel('u', 'Interpreter','latex')
    print(h,'-dpng', path);
    close(h);
end

