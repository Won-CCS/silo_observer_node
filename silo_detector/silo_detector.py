import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import TransformStamped

NUMBER_OF_SILOS = 1
SILO_CAPACITY = 3

class Vector:
    x = 0
    y = 0
    z = 0

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class Coordinate:
    x = 0
    y = 0
    z = 0
    theta = 0

    def __init__(self, x, y, z, theta):
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta

class Camera:
    # 外部パラメータ
    rmat = np.array([[0, -1, 0], 
                     [0, 0, -1], 
                     [1, 0, 0]], dtype=np.float32)
    tvec = np.array([[0], [0], [0]], dtype=np.float32)
    # 内部パラメータ
    # Logiccolのc922
    # focal_length = 0.001
    # k1 = 0.03520446031433724
    # k2 = -0.2621147575929849
    # p1 = 0.004920860634892838
    # p2 = 0.007969216065437846
    # k3 = -0.1871326332054414
    # ELECOMカメラ
    focal_length = 0.00415
    k1 = 0.0970794248992087
    k2 = 0.46852992832469376
    p1 = -0.0027402493655248367
    p2 = -0.002055751211595421
    k3 = -12.963018944283235
    # 内部パラメータ行列
    # Logiccolのc922
    # camera_matrix = cv2.Mat(np.array([[1422.092372366652, 0.0, 994.0655146868652], [0.0, 1422.6878709473806, 521.7945002394441], [0.0, 0.0, 1.0]], dtype=np.float32))
    # ELECOMカメラ
    camera_matrix = np.array([[970.7808694146526, 0.0, 385.0122379475739], [0.0, 970.1929411781452, 230.67852825871415], [0.0, 0.0, 1.0]], dtype=np.float32)


class Silo:
    top_coordinate = np.array([[0], [0], [525]], dtype=np.float32)
    bottom_coordinate = np.array([[0], [0], [100]], dtype=np.float32)
    top_camera_coordinate = np.array([0, 0, 0], dtype = np.float32)
    bottom_camera_coordinate = np.array([0, 0, 0], dtype = np.float32)
    top_image_coordinate = np.array([0, 0], dtype = np.float32)
    bottom_image_coordinate = np.array([0, 0], dtype = np.float32)

    width = 250
    height = 425
    balls = [None, None, None]
    detection_flag = False

    def __init__(self, coordinate):
        self.top_coordinate = np.array([[coordinate.x], [coordinate.y], [525]], dtype=np.float32)
        self.bottom_coordinate = np.array([[coordinate.x], [coordinate.y], [100]], dtype=np.float32)

class SiloObserver(Node):
    def __init__(self):
        super().__init__('silo_observer_node')
        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
          'target_frame', 'base_link').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # self.publisher = self.create_publisher(String, 'number_of_balls', 1)

        camera = Camera()

        # on_timer関数を1秒ごとに実行
        # self.timer = self.create_timer(1.0, lambda: self.on_timer(camera))

        self.observe_silos(camera)

    
    def on_timer(self, camera):
        # transformの取得に使用する変数にフレーム名を格納する。
        from_frame_rel = self.target_frame
        to_frame_rel = 'odom'

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())

            # self.get_logger().info(f'got transform {to_frame_rel} to {from_frame_rel}')
            
            # カメラの外部パラメータ(ここは機体座標と、機体の姿勢から逐次的に求める必要がある)
            quat = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
            camera.rmat = quaternion_to_rotation_matrix(quat) * np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=np.float32)
            # camera.tvec = np.array([[0], [0], [50]], dtype=np.float32)
            camera.tvec = np.matmul(camera.rmat, np.array([[t.transform.translation.x], [t.transform.translation.y], [t.transform.translation.z]], dtype=np.float32)) + np.array([[0], [0], [50]], dtype=np.float32)

            # self.get_logger().info(f'{camera.rmat}')
            
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        # self.msg = String()
        

        # number_of_balls = detect_dominant_color_ratio(self, t)


        # self.publisher.publish(msg)


    def observe_silos(self, camera):
        # カメラのキャプチャを開始
        cap = cv2.VideoCapture(0)
        while True:
            # カメラからフレームを取得
            ret, frame = cap.read()
            if not ret:
                break

            # サイロの座標
            silos = [Silo(Coordinate(6000, 11500, 100, 0)), Silo(Coordinate(6000, 10750, 100, 0)), Silo(Coordinate(6000, 10000, 100, 0)), Silo(Coordinate(6000, 9250, 100, 0)), Silo(Coordinate(6000, 8500, 100, 0))]
            # silos = [Silo(Coordinate(4000, 1000, 0, 0))]

            # カメラ画像上のサイロの頂点の座標を取得
            silos = world_to_camera_coordinate(silos, camera)
            silos = camera_to_image_coordinate(silos, camera)

            # 画像を切り取ってボール検出
            cut_imgs = cut_image(silos, frame)
            silos = detect_balls(cut_imgs, silos, self)

            cv2.namedWindow('cut', cv2.WINDOW_NORMAL)
            cv2.imshow('cut', cut_imgs[0])

            # for i in range(NUMBER_OF_SILOS):
            #     self.get_logger().info(f'{silos[i].balls}')

            # 画像を表示
            for silo in silos:
                cv2.circle(frame, (int(silo.bottom_image_coordinate[0][0]), int(silo.bottom_image_coordinate[1][0])), 10, (0, 0, 255), thickness=cv2.FILLED)
                cv2.circle(frame, (int(silo.top_image_coordinate[0][0]), int(silo.top_image_coordinate[1][0])), 10, (0, 0, 255), thickness=cv2.FILLED)
            cv2.namedWindow('Detected', cv2.WINDOW_NORMAL)
            cv2.imshow('Detected', frame)

            # 'q' キーでループを終了
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        #メモリを解放して終了するためのコマンド
        cap.release()
        cv2.destroyAllWindows()
        

def quaternion_to_rotation_matrix(quat):
    x, y, z, w = quat
    rmat = np.array([[1 - 2 * y ** 2 - 2 * z ** 2, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w], [2 * x * y + 2 * z * w, 1 - 2 * x ** 2 - 2 * z ** 2, 2 * y * z - 2 * x * w], [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x ** 2 - 2 * y ** 2]], dtype=np.float32)
    
    return rmat

# カメラ画像上のサイロの頂点の座標を取得
def world_to_camera_coordinate(silos: Silo, camera: Camera):
    for silo in silos:
        silo.bottom_camera_coordinate = np.matmul(camera.rmat, silo.bottom_coordinate) + camera.tvec
        silo.top_camera_coordinate = np.matmul(camera.rmat, silo.top_coordinate) + camera.tvec

    return silos

#歪みを考慮しない座標変換
# def camera_to_image_coordinate(bottom_camera_coordinate, top_camera_coordinate, camera: Camera):
#     bottom_image_coordinate = np.matmul(camera.camera_matrix, bottom_camera_coordinate / bottom_camera_coordinate[2])
#     top_image_coordinate = np.matmul(camera.camera_matrix, top_camera_coordinate / top_camera_coordinate[2])
#     # print(camera.camera_matrix * top_camera_coordinate)

#     return bottom_image_coordinate, top_image_coordinate

#歪みを考慮した座標変換
def camera_to_image_coordinate(silos: Silo, camera: Camera):
    for silo in silos:
        bottom_x = silo.bottom_camera_coordinate[0] / silo.bottom_camera_coordinate[2]
        bottom_y = silo.bottom_camera_coordinate[1] / silo.bottom_camera_coordinate[2]
        r_squared_2 = bottom_x ** 2 + bottom_y ** 2
        silo.bottom_camera_coordinate[0] = bottom_x * (1 + camera.k1 * r_squared_2 + camera.k2 * r_squared_2 ** 2 + camera.k3 * r_squared_2 ** 3) + 2 * camera.p1 * bottom_x * bottom_y + camera.p2 * (r_squared_2 + 2 * bottom_x ** 2)
        silo.bottom_camera_coordinate[1] = bottom_y * (1 + camera.k1 * r_squared_2 + camera.k2 * r_squared_2 ** 2 + camera.k3 * r_squared_2 ** 3) + 2 * camera.p2 * bottom_x * bottom_y + camera.p1 * (r_squared_2 + 2 * bottom_y ** 2)
        silo.bottom_camera_coordinate[2] = 1
        
        top_x = silo.top_camera_coordinate[0] / silo.top_camera_coordinate[2]
        top_y = silo.top_camera_coordinate[1] / silo.top_camera_coordinate[2]
        r_squared_2 = top_x ** 2 + top_y ** 2
        silo.top_camera_coordinate[0] = top_x * (1 + camera.k1 * r_squared_2 + camera.k2 * r_squared_2 ** 2 + camera.k3 * r_squared_2 ** 3) + 2 * camera.p1 * top_x * top_y + camera.p2 * (r_squared_2 + 2 * top_x ** 2)
        silo.top_camera_coordinate[1] = top_y * (1 + camera.k1 * r_squared_2 + camera.k2 * r_squared_2 ** 2 + camera.k3 * r_squared_2 ** 3) + 2 * camera.p2 * top_x * top_y + camera.p1 * (r_squared_2 + 2 * top_y ** 2)
        silo.top_camera_coordinate[2] = 1

        silo.bottom_image_coordinate = np.matmul(camera.camera_matrix, silo.bottom_camera_coordinate)
        silo.top_image_coordinate = np.matmul(camera.camera_matrix, silo.top_camera_coordinate)

    return silos

def cut_image(silos: Silo, image):
    cut_imgs = []
    for silo in silos:
        if(0 < silo.top_image_coordinate[0] < 640 and 0 < silo.top_image_coordinate[1] < 480 and 0 < silo.bottom_image_coordinate[0] < 640 and 0 < silo.bottom_image_coordinate[1] < 480):
            silo.detection_flag = True
            cut_imgs.append(image[int(silo.top_image_coordinate[1]) : int(silo.bottom_image_coordinate[1]), int(silo.bottom_image_coordinate[0] - 20) : int(silo.bottom_image_coordinate[0] + 20)])
        else:
            silo.detection_flag = False
    
    return cut_imgs

def detect_balls(cut_imgs, silos, node):
    region_height = []
    for cut_img in cut_imgs:        
        # 画像を縦方向に3分割
        height, width = cut_img.shape[:2]
        region_height.append(height // 3)

    # 各領域のHSV割合を保存するリスト
    red_ratios = []
    blue_ratios = []

    for i in range(NUMBER_OF_SILOS):
        if silos[i].detection_flag == True:
            for j in range(SILO_CAPACITY):
                if(silos[i].balls[j] == None):
                    # 各領域の範囲を計算
                    start_y = j * region_height[i]
                    end_y = (j + 1) * region_height[i]

                    # 領域を切り取り
                    region = cut_imgs[i][start_y:end_y, :]

                    # BGR形式からHSV形式に変換
                    region_hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)

                    # 赤色および青色のHSV範囲（調整が必要な場合は変更）
                    red_lower = np.array([0, 100, 100])
                    red_upper = np.array([10, 255, 255])
                    blue_lower = np.array([100, 70, 50])
                    blue_upper = np.array([140, 255, 255])

                    # 赤色と青色の領域を抽出
                    red_mask = cv2.inRange(region_hsv, red_lower, red_upper)
                    blue_mask = cv2.inRange(region_hsv, blue_lower, blue_upper)

                    # 赤色と青色のピクセル数をカウント
                    red_pixels = cv2.countNonZero(red_mask)
                    blue_pixels = cv2.countNonZero(blue_mask)

                    # 画像内のピクセル数
                    total_pixels = region_hsv.shape[0] * region_hsv.shape[1]

                    # 赤色と青色の割合を計算
                    red_ratio = red_pixels / total_pixels
                    blue_ratio = blue_pixels / total_pixels

                    # 各領域の赤色と青色の割合を保存
                    red_ratios.append(red_ratio)
                    blue_ratios.append(blue_ratio)
                    
                    if red_ratios[j] > 0.4:
                        silos[i].balls[j] = "red"
                        # print(f"Region {j + 1}: Dominant Color - Red")
                    elif blue_ratios[j] > 0.4:
                        silos[i].balls[j] = "blue"
                        # print(f"Region {j + 1}: Dominant Color - Blue")
                    else:
                        silos[i].balls[j] = "None"
                        # print(f"Region {j + 1}: No dominant color or not dominant enough")
                
    return silos

def main(args=None):
    rclpy.init(args=args)

    silo_observer = SiloObserver()

    try:
        rclpy.spin(silo_observer)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()