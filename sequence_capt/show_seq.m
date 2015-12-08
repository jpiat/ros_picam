cc = hsv(12);
files = dir('*.pgm');
load('IMU.log');
acc = IMU(:, 2:4)*(4/32768);
gyro = IMU(:,5:7)*(500/32768);
acc_plot = figure;
hold all
for i=1:3
    plot(IMU(:, 1), acc(:,i),'color',cc(i,:));
end
gyro_plot = figure;
hold all
for i=1:3
    plot(IMU(:, 1),gyro(:,i) ,'color',cc(i+4,:));
end
image_plot = figure ;
start_index = 1;
end_index = 1;
for file = files'
    [pathstr,name,ext] = fileparts(file.name) ;
    image_time = str2double(name);
    subplot(2, 1, 1);
    end_index = start_index ;
    while IMU(end_index, 1) < image_time
        end_index  = end_index + 1 ;
    end
    axis equal
    image = imread(file.name);
    imshow(image);
    subplot(2, 1, 2);
    plot(IMU(start_index:end_index, 1),gyro(start_index:end_index,1) ,'color',cc(1,:));
    start_index = end_index;
    pause ;
    %csv = load(file.name)
    % Do some stuff
end