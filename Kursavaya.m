% команды для очищения консоли перед началом работы прораммы
clear, clc, clf

% заданные параметры подвижного устройства и территории
speed = 18;
work_time = 3.5;
charge_time = 27/60;
start = [-3 9]; % zeros(1, 2);
points = zeros(8, 2);
finish = [10 8]; %zeros(1, 2)
scale = 15;

% дополнительные характеристики
drive = speed*work_time;
scale_drive = drive/scale;

% функция для связи пользователя с программой
while true
    clc
    disp("1. Ввод данных")
    disp("2. Построение матрицы смежности")
    disp("3. Построение матрицы кратчайших путей")
    disp("4. Поиск кратчайшего пути")
    disp("5. Построение графика перемещений")
    disp("6. Вывод NMEA сообщений")
    disp("7. Выход")
    choise = input("Введите номер команды ");
    switch choise
        case 1
            clc
            map = build_map
            pause
        case 2
            clc
            da_net_map = build_da_net_map(map, scale, scale_drive)
            pause
        case 3
            clc
            length_map = build_length_map(map, da_net_map, scale)
            pause
        case 4
            clc
            [D, ind, way_ind] = dijcstara(map, length_map);
            pause
        case 5
            build_graph(map, da_net_map, way_ind)
        case 6
            clc
            nmea_vivod(map, length_map, way_ind, speed, charge_time, work_time)
            pause
        case 7
            break
        otherwise
            disp("Неверная команда")
    end
end

% считывание данных о точках из файла, ввод начальной и конечной точек
function map = build_map
    points = readmatrix("Входные данные.txt");
    start(1, 1) = input("Х начальное = ");
    start(1, 2) = input("У начальное = ");
    finish(1, 1) = input("Х конечное = ");
    finish(1, 2) = input("У конечное = ");
    map = [start; points; finish]; % массив всех точек
end

% построение матрицы смежности
function da_net_map = build_da_net_map(map, scale, scale_drive)
    da_net_map = zeros(length(map), length(map));
    for i = 1:length(map)
        for j = (i+1):length(map)  
            l = sqrt(power(map(i, 1)-map(j, 1),2)+power(map(i, 2)-map(j, 2),2));
            if l <= scale_drive
                da_net_map(i, j) = 1;
                da_net_map(j, i) = 1;
            end
        end
    end
end 

% построение матрицы минимальных путей
function length_map = build_length_map(map, da_net_map, scale)
    length_map = zeros(length(map), length(map));
    for i = 1:length(map)
        for j = (i+1):length(map)        
            if da_net_map(i, j) == 1
                l = sqrt(power(map(i, 1)-map(j, 1),2)+power(map(i, 2)-map(j, 2),2));
                length_map(i,j) = l*scale;
                length_map(j,i) = l*scale;
            else
                length_map(i,j) = inf;
                length_map(j,i) = inf;
            end
        end
    end
end

% функция для поиска оптимального пути (метод Дейкстры)
function [D, ind, way_ind] = dijcstara(map, length_map)
    D1 = inf*ones(length(map), 1);
    D2 = zeros(length(map), length(map)-1);
    D = [D1, D2]; % матрица из векторов D (по столбцам)
    D(1, 1) = 0;
    fprintf("D1 = ")
    disp(D(:, 1)')
    
    ind = zeros(1, length(map)); % вектор индексов кратчайших расстояний
    ind(1) = 1;
    k = 1;
    fprintf("ind = ")
    dind = ind;
    dind(dind == 0) = [];
    disp(dind)
    fprintf("\n")
    
    for j = 2:length(map)
        minind_znach = inf;
        minind = inf;
        for i = 2:length(map)
             if ismember(i, ind) == 1 % если индекс элемента уже есть то переносим элемент в новый столбец
                 D(i, j) = D(i, j-1);
             else
                 minmax = D(k, j) + length_map(k, i); 
                 if minmax < minind_znach; % поиск минимального элемента (из тех, чьего индекса еще нет)
                     minind_znach = minmax;
                     minind = i;
                 end
                 if minmax < D(i, j-1) && minmax ~= 0 % записываем новую длину в новый столбец если она меньше уже имеющейся
                     D(i, j) = minmax;
                 else
                     D(i, j) = D(i, j-1);
                 end
             end
        end
        ind(j) = minind; % записываем новый индекс минимального элемента
        k = k + 1;
        D_j = D(:, j)';
        fprintf("D%d = ", j)
        disp(D_j)
        fprintf("ind = ")
        dind = ind;
        dind(dind == 0) = []; 
        disp(dind)
        fprintf("\n")
    end
    %D % последний столбец показывает наименьшую длину пути до каждого пункта
    %ind

    % восстановление кратчайшего пути по точкам
    way_ind = zeros(1, length(ind)); % вектор индексов посещенных пунктов
    way_ind(length(way_ind)) = ind(length(ind)); % последний элемент нового массива равен индексу конечного пункта
    min_ways = D(:,length(map)); % последний столбец вектора D
    
    i = length(way_ind);
    j = 0;
    while (i-j) > 0
        for j = 1:(i-1)
            if abs(min_ways(i) - min_ways(i-j) - length_map(i, i-j)) <= 0.0001   
                way_ind(i-1) = i-j;
                i = i - j;
                break
            end
        end
    end
    way_ind; % вектор индексов посещенных пунктов
    way_ind(way_ind == 0) = [] % удаление лишних нулей из вектора
end

% построение графа возможных перемещений
function graphic = build_graph(map, da_net_map, way_ind)
    hold on
    axis('equal')
    for i = 1:length(map)
        for j = i+1:length(map)
            if da_net_map(i, j) == 1
                plot([map(i, 1), map(j, 1)], [map(i, 2), map(j, 2)], '--', 'Color', 'k'); 
            end
        end
    end

    % построение графика минимального пути
    a = zeros(length(way_ind), 1);
    b = zeros(length(way_ind), 1);
    for i = 1:length(way_ind)
        a(i) = map(way_ind(i), 1);
        b(i) = map(way_ind(i), 2);
    end
     
end

% рассчет времени пути и других составляющих + вывод NMEA сообщения
function vivod = nmea_vivod(map, length_map, way_ind, speed, charge_time, work_time)
    way_time = 0;
    charging_time = 0;
    s2 = "N"; % движение продолжается
    
    way_to_next = length_map(way_ind(1), way_ind(2));
    ugol = rad2deg(atan((map(way_ind(2), 2)-map(way_ind(1), 2))/(map(way_ind(2), 1)-map(way_ind(1), 1))));
    if ugol == 0
        s1 = "N"; 
    else
        s 
    end
    if ugol < 0
        ugol = 360 + ugol;
    end
    fprintf('$UTHDG,0,00.00,%0.2f,%s,%0.1f,%s', way_to_next, s1, ugol, s2) % сообщения для первого пункта
    fprintf("\n")
    
    for i = 1:(length(way_ind) - 1)
        if  
            way_to_next = length_map(way_ind(i+1), way_ind(i+2));
            ugol = rad2deg(atan((map(way_ind(i+2), 2)-map(way_ind(i+1), 2))/(map(way_ind(i+2), 1)-map(way_ind(i+1), 1))));
        else
            way_to_next = 0;
            ugol  = 0;
        end
        time_to_next = length_map(way_ind(i), way_ind(i+1))/speed;
        time_obs = time_to_next + way_time + charging_time;
        time_hrs = floor(time_obs);
        time_min = (time_obs - time_hrs)*60;
        way_time = way_time + time_obs;
        charging_time = charge_time*time_to_next/work_time;
        if ugol < 0
            ugol = 360 + ugol;
        end
        if ugol == 0
            s1 = "N"; % поворота нет
        else
            s1 = "T"; % есть поворот
        end
        if i == 4
            s2 = "E"; % движение закончено
        end
        fprintf('$UTHDG,%d,%0.1f,%0.2f,%s,%0.1f,%s', time_hrs, time_min, way_to_next, s1, ugol, s2)
        fprintf("\n")
    end
end