# deliverobotino
Robot that follows you and brings tools

# Цель
Автономное движение робота Robotino к месту (человеку), обозначенному с помощью ARTag, с целью принятия или доставки инструмента и последующего отъезда на "базу" по звуковому сигналу.

# Используемые технологии
- ROS Kinetic
- Robotino node
- XBOX Kinect
- ar_track_alvar
- audio_capture
- gmapping
- xarco

- move_base
- usb_cam
# Описание
Робот предназначен для следования к обозначенному AR-тэгом месту с помощью пакета move_base и ASUS Xtion, определяя его положение, используя ar_track_alvar и usb_cam. Выполнив свою задачу (прием или передача инструмента/другого объекта), робот возвращается на "базу", получив звуковой сигнал в виде хлопка (audio_capture).

# Установка
- git clone https://github.com/betapl3b/deliverobotino
- apt-get update
- apt-get install ros-kinetic-tf2
- apt-get install ros-kinetic-ar-track-alvar
- apt-get install ros-kinetic-move-base 
- apt-get install ros-kinetic-gmapping
- apt-get install ros-kinetic-gazebo-ros
- apt-get install ros-kinetic-xacro
- apt-get install ros-kinetic-audio-capture 
- apt-get install ros-kinetic-depthimage-to-laserscan

Далее в директории project_simulation/worlds в файле room_filled и room.world необходимо заменить строчки:

<uri>model:///home/beta/catkin_ws/src/project_simulation/models/marker/materials/scripts</uri>
<uri>model:///home/beta/catkin_ws/src/project_simulation/models/marker/materials/textures</uri>

на соответствующие пути на своем пк (в основном необходимо изменить имя пользователя и название воркспейса).

- Запуск симуляции производится командой roslaunch project_simulation start.launch
- Запуск контроллера произовдится командой rosrun project_simulation go.py
- Тестовые контроллеры также находятся в папке scripts.
