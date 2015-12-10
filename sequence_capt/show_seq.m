cc = hsv(12);
files = dir('*.pgm');
load('IMU.log');
acc = IMU(:, 2:4)*(4/32768);
gyro = IMU(:,5:7)*(500/32768);
time_plot = figure('Name', 'time');
plot(IMU(:, 1));
acc_plot = figure('Name','Accelero');
hold all
for i=1:3
    plot(IMU(:, 1), acc(:,i),'color',cc(i,:));
end
gyro_plot = figure('Name','Gyro');
hold all
for i=1:3
    plot(IMU(:, 1),gyro(:,i) ,'color',cc(i+4,:));
end
image_plot = figure('Name', 'Image and associated IMU data') ;
start_index = 1;
end_index = 1;
for file = files'
    [pathstr,name,ext] = fileparts(file.name) ;
    image_time = str2double(name);
    img_plot = subplot(3, 1, 1);
    end_index = start_index ;
    while IMU(end_index, 1) < image_time
        end_index  = end_index + 1 ;
    end
    nb_imu_samples = end_index - start_index
    image = imread(file.name);
    imshow(image);
    handle_acc_plot = subplot(3, 1, 2);
    hold all
    for i=1:3
        plot(IMU(start_index:end_index, 1), acc(start_index:end_index,i),'color',cc(i,:));
    end
    handle_gyro_plot = subplot(3, 1, 3);
    hold all
    for i=1:3
        plot(IMU(start_index:end_index, 1),gyro(start_index:end_index,i) ,'color',cc(i+4,:));
    end
    start_index = end_index;
    pause ;
    delete(handle_acc_plot);
    delete(handle_gyro_plot);
end