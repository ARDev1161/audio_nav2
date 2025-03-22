cmake_minimum_required(VERSION 3.5)
project(odas_classifier_msgs)

# Найдите необходимые пакеты
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Укажите файлы сообщений, которые будут сгенерированы
set(msg_files
  "msg/SoundSourceClassification.msg"
)

# Указываем зависимости для интерфейсов
set(rosidl_interface_packages
  std_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES ${rosidl_interface_packages}
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
