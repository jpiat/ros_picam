files = dir('*.pgm');
load(IMU.log)
for file = files'
    [pathstr,name,ext] = fileparts(file.name) ;
    image_time = str2double(name)
    %csv = load(file.name)
    % Do some stuff
end