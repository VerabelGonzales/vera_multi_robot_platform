#!/usr/bin/env python3

import numpy as np
import cv2
import rospy
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from geometry_msgs.msg import Quaternion
import tf
import math
from filterpy.kalman import KalmanFilter
from collections import deque

# Configuraciones globales
ARUCO_DICT = {
    # Diccionarios ArUco disponibles
    # Cada clave del diccionario es un tipo de marcador ArUco
    # y su valor correspondiente es el identificador en OpenCV
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

ROBOT_MARKERS = {
    # Asignación de identificadores de marcadores ArUco a nombres de robots y objetos
    0: "robot_1",  # ID 1 corresponde a robot_1
    1: "robot_2",  # ID 2 corresponde a robot_2
    4: "robot_3",  # ID 3 corresponde a robot_3
    5: "objeto",  # ID 3 corresponde a robot_3
}

# Matriz de calibración intrínseca de la cámara y coeficientes de distorsión
INSTRINSIC_CAMERA_MATRIX = np.array(((960.483009, 0.000000, 616.668137), 
                                    (0.000000, 961.565864, 342.199999), 
                                    (0.000000, 0.000000, 1.000000)))

DISTORTION_COEFF = np.array((-0.017074, -0.056332, -0.000439, -0.003168, 0.000000))
ARUCO_TYPE = "DICT_ARUCO_ORIGINAL" # Tipo de marcador ArUco utilizado

# Inicializar variables globales
block_size = 27
c_value = 2

def initialize_camera():
    # Inicializa la cámara y configura la resolución
    cap = cv2.VideoCapture(2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    return cap

def create_settings_window():
    # Crea una ventana para ajustes y agrega deslizadores para la configuración
    cv2.namedWindow("Settings", cv2.WINDOW_NORMAL)
    cv2.createTrackbar("Block Size", "Settings", block_size, 200, on_block_size_change)
    cv2.createTrackbar("C Value", "Settings", c_value, 100, on_c_value_change)
    cv2.resizeWindow("Settings", 640, 100)  # Ajusta dimensiones según necesidad

def on_block_size_change(val):
    # Callback para actualizar el tamaño del bloque en la binarización adaptativa
    global block_size
    block_size = val
    if block_size % 2 == 0:
        block_size += 1

def on_c_value_change(val):
    # Callback para actualizar el valor C en la binarización adaptativa
    global c_value
    c_value = val

def rotation_vector_to_euler_angles(rvec):
    # Convertir el vector de rotación a una matriz de rotación
    R, _ = cv2.Rodrigues(rvec)

    # Calcular los ángulos de Euler a partir de la matriz de rotación
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else:
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z]) #* (180.0 / math.pi)  # Convertir a grados

def create_kalman_filter(dt=0.1):
    kf = KalmanFilter(dim_x=4, dim_z=2)
    
    # Estado inicial
    kf.x = np.array([0., 0., 0., 0.])  # [x, y, dx, dy]

    # Matriz de transición de estado
    kf.F = np.array([[1, 0, dt, 0],
                     [0, 1, 0, dt],
                     [0, 0, 1,  0],
                     [0, 0, 0,  1]])

    # Matriz de medición
    kf.H = np.array([[1, 0, 0, 0],
                     [0, 1, 0, 0]])

    # Covarianza del ruido del proceso
    kf.Q = np.eye(4) * 0.01

    # Covarianza del ruido de medición
    kf.R = np.eye(2) * 0.1

    # Covarianza del estado inicial
    kf.P = np.eye(4) * 1

    return kf

def normalize_quaternion(quaternion):
    norm = math.sqrt(quaternion.x**2 + quaternion.y**2 + quaternion.z**2 + quaternion.w**2)
    quaternion.x /= norm
    quaternion.y /= norm
    quaternion.z /= norm
    quaternion.w /= norm
    return quaternion

def pose_estimation(frame, aruco_dict):
    # Conversión a escala de grises y binarización
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                   cv2.THRESH_BINARY, block_size, c_value)
    kernel_size = 3
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    binary_eroded = cv2.erode(binary, kernel, iterations=1)
    binary_dilated = cv2.dilate(binary_eroded, kernel, iterations=1)

    # Detección de marcadores ArUco
    parameters = cv2.aruco.DetectorParameters_create()
    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(
        binary, aruco_dict, parameters=parameters,
        cameraMatrix=INSTRINSIC_CAMERA_MATRIX, distCoeff=DISTORTION_COEFF)
    
    euler_angles = None
    rvecs, tvecs = [], []

    if ids is not None:
        ids = ids.flatten()
        for i, corner in enumerate(corners):
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, 0.10, 
                                    INSTRINSIC_CAMERA_MATRIX, DISTORTION_COEFF)
            rvecs.append(rvec)
            tvecs.append(tvec)

            cv2.aruco.drawDetectedMarkers(frame, corners)
            cv2.aruco.drawAxis(frame, INSTRINSIC_CAMERA_MATRIX, DISTORTION_COEFF, 
                               rvec, tvec, 0.1)
            
            # Imprimir el ID del marcador
            #marker_id = ids[i]
            #print("Detectado marcador ArUco con ID:", marker_id)
            
            #euler_angles = rotation_vector_to_euler_angles(rvec[0][0])
            #print("Euler Angles (Degrees):", euler_angles)

    # Conversión de la imagen binarizada a color para coincidir con la imagen de la cámara
    binary_dilated_colored = cv2.cvtColor(binary_dilated, cv2.COLOR_GRAY2BGR)

    # Redimensionar para que todas las imágenes tengan el mismo tamaño
    frame_resized = cv2.resize(frame, (640, 360))
    binary_dilated_resized = cv2.resize(binary_dilated_colored, (640, 360))

    # Concatenar imágenes horizontalmente
    concatenated_images = cv2.vconcat([frame_resized, binary_dilated_resized])

    return concatenated_images, rvecs, tvecs, ids

kalman_filters = {robot: create_kalman_filter() for robot in ROBOT_MARKERS.values()}

# Configuración del filtro de media móvil
N = 10  # Número de muestras para el promedio
history = {robot: {'positions': deque(maxlen=N), 'orientations': deque(maxlen=N)}
           for robot in ROBOT_MARKERS.values()}
           
def publish_odometry(odom_pubs, rvec, tvec, marker_id, odomBroadcaster, base_frame_id, odom_frame_id):
    if marker_id in ROBOT_MARKERS:
        robot_topic = ROBOT_MARKERS[marker_id]
        if robot_topic in odom_pubs:
            # Asegúrate de que rvec y tvec son arrays de una sola dimensión
            rvec = rvec.flatten()
            tvec = tvec.flatten()

            quaternion = Quaternion()
            
            if tvec.size < 2:
                print(f"Advertencia: tvec tiene un tamaño inesperado {tvec.size}. Se esperaba al menos 2.")
                return
            
            # Convertir rvec de vector de rotación a ángulos de Euler y luego a cuaternión
            euler_angles = rotation_vector_to_euler_angles(rvec.flatten())
            quaternion_ini = tf.transformations.quaternion_from_euler(0.0, 0.0, euler_angles[2])
            
            # Actualizar el filtro de Kalman
            kf = kalman_filters[robot_topic]
            kf.predict()
            kf.update(np.array([tvec[0], tvec[1]]))

            # Obtener la posición filtrada por el filtro de Kalman
            filtered_tvec = kf.x[:2]

            # Actualizar historial para el filtro de media móvil
            history[robot_topic]['positions'].appendleft(filtered_tvec)
            history[robot_topic]['orientations'].appendleft(quaternion_ini)

            # Calcular la media de las posiciones y orientaciones
            avg_position = np.mean(history[robot_topic]['positions'], axis=0)
            avg_orientation = np.mean(history[robot_topic]['orientations'], axis=0)

            # Asegúrate de que los frame IDs incluyan el namespace
            namespaced_base_frame_id = f"{robot_topic}/{base_frame_id}"
            namespaced_odom_frame_id = f"{robot_topic}/{odom_frame_id}"

            odomBroadcaster.sendTransform(
                (avg_position[0], avg_position[1], 0),
                (quaternion_ini[0], quaternion_ini[1], quaternion_ini[2], quaternion_ini[3]),
                rospy.Time.now(),
                namespaced_base_frame_id,
                odom_frame_id
                )

            # Crear y publicar el mensaje de odometría utilizando los promedios
            odom_msg = Odometry()
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = "/odom"
            odom_msg.child_frame_id = namespaced_base_frame_id
            odom_msg.pose.pose.position.x = avg_position[0]
            odom_msg.pose.pose.position.y = avg_position[1]
            odom_msg.pose.pose.position.z = 0  # Asumiendo una posición Z plana
            odom_msg.pose.pose.orientation = Quaternion(*avg_orientation)

            # Publicar el mensaje de odometría
            odom_pubs[robot_topic].publish(odom_msg)

def main():
    # Función principal
    rospy.init_node('aruco_pose_publisher')
    # Configuración de los publicadores de ROS y el broadcaster de transformadas
    odom_pubs = {robot: rospy.Publisher(f'/{robot}/odom', Odometry, queue_size=10) for robot in ROBOT_MARKERS.values()}
    odomBroadcaster = TransformBroadcaster()

    base_frame_id = rospy.get_param('~base_frame_id', 'base_footprint')
    odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
    
    cap = initialize_camera()
    create_settings_window()
    aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT[ARUCO_TYPE])

    while cap.isOpened() and not rospy.is_shutdown():
        # Captura de imagen y procesamiento
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        concatenated_images, rvecs, tvecs, ids = pose_estimation(frame, aruco_dict)
        
        # Actualizar todos los filtros Kalman (paso de predicción)
        for kf in kalman_filters.values():
            kf.predict()

        if ids is not None:
            for i, marker_id in enumerate(ids):
                publish_odometry(odom_pubs, rvecs[i][0], tvecs[i][0], marker_id, odomBroadcaster, base_frame_id, odom_frame_id)

        # Mostrar todas las imágenes en una ventana
        cv2.imshow('Aruco Detector', concatenated_images)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
