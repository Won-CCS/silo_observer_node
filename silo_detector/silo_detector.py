import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import TransformStamped

NUMBER_OF_SILOS = 5
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
    #外部パラメータ
    rmat = np.array([[0, 0, 1], 
                     [-1, 0, 0], 
                     [0, -1, 0]], dtype=np.float32)
    tvec = np.array([[0], [0], [0]], dtype=np.float32)
    #内部パラメータ
    focal_length = 0.001
    k1 = 0.03520446031433724
    k2 = -0.2621147575929849
    p1 = 0.004920860634892838
    p2 = 0.007969216065437846
    k3 = -0.1871326332054414
    #内部パラメータ行列(c922を今は使っている)
    camera_matrix = cv2.Mat(np.array([[1422.092372366652, 0.0, 994.0655146868652], [0.0, 1422.6878709473806, 521.7945002394441], [0.0, 0.0, 1.0]], dtype=np.float32))
    coordinate = Coordinate(0, 0, 0, 0)

    #コンストラクタ
    def __init__(self, coordinate):
        self.coordinate = coordinate

class Silo:
    top_coordinate = cv2.Mat(np.array([[0], [0], [525]], dtype=np.float32))
    bottom_coordinate = cv2.Mat(np.array([[0], [0], [100]], dtype=np.float32))
    width = 250
    height = 425
    balls = [None, None, None]

    def __init__(self, coordinate):
        self.top_coordinate = cv2.Mat(np.array([[coordinate.x], [coordinate.y], [525]], dtype=np.float32))
        self.bottom_coordinate = cv2.Mat(np.array([[coordinate.x], [coordinate.y], [100]], dtype=np.float32))

class SiloObserver(Node):
    def __init__(self):
        super().__init__('silo_observer_node')
        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
          'target_frame', 'base_link').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # self.publisher = self.create_publisher(String, 'number_of_balls', 1)

        camera = Camera(Coordinate(0, 0, 0, 0))

        # on_timer関数を1秒ごとに実行
        self.timer = self.create_timer(1.0, lambda: self.on_timer(camera))

        # self.observe_silos(camera)

    
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
            camera.x = t.transform.translation.x
            camera.y = t.transform.translation.y 
            camera.z = t.transform.translation.z
            quat = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
            camera.rmat = quaternion_to_rotation_matrix(quat)
            camera.tvec = np.array([[0], [10], [0]], dtype=np.float32)

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

            # カメラ画像上のサイロの頂点の座標を取得
            bottom_camera_coordinates, top_camera_coordinates = world_to_camera_coordinate(silos, camera)
            bottom_image_coordinates, top_image_coordinates = camera_to_image_coordinate(bottom_camera_coordinates, top_camera_coordinates, camera)

            # 画像を切り取る
            cut_imgs = cut_image(bottom_image_coordinates, top_image_coordinates, frame)
            silos = detect_balls(cut_imgs, silos)

            # 画像を表示
            for i in range(NUMBER_OF_SILOS):
                cv2.circle(frame, (int(bottom_image_coordinates[i][0][0]), int(bottom_image_coordinates[i][1][0])), 10, (0, 0, 255), thickness=cv2.FILLED)
                cv2.circle(frame, (int(top_image_coordinates[i][0][0]), int(top_image_coordinates[i][1][0])), 10, (0, 0, 255), thickness=cv2.FILLED)
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
def world_to_camera_coordinate(silos, camera: Camera):
    bottom_camera_coordinates = []
    top_camera_coordinates = []
    for silo in silos:
        bottom_camera_coordinates.append(np.matmul(camera.rmat, silo.bottom_coordinate) + camera.tvec)
        top_camera_coordinates.append(np.matmul(camera.rmat, silo.top_coordinate) + camera.tvec)

    return bottom_camera_coordinates, top_camera_coordinates

#歪みを考慮しない座標変換
# def camera_to_image_coordinate(bottom_camera_coordinate, top_camera_coordinate, camera: Camera):
#     bottom_image_coordinate = np.matmul(camera.camera_matrix, bottom_camera_coordinate / bottom_camera_coordinate[2])
#     top_image_coordinate = np.matmul(camera.camera_matrix, top_camera_coordinate / top_camera_coordinate[2])
#     # print(camera.camera_matrix * top_camera_coordinate)

#     return bottom_image_coordinate, top_image_coordinate

#歪みを考慮した座標変換
def camera_to_image_coordinate(bottom_camera_coordinates, top_camera_coordinates, camera: Camera):
    bottom_image_coordinates = [[], [], []]
    top_image_coordinates = [[], [], []]
    for i in range(len(bottom_camera_coordinates)):
        bottom_x = bottom_camera_coordinates[i][0] / bottom_camera_coordinates[i][2]
        bottom_y = bottom_camera_coordinates[i][1] / bottom_camera_coordinates[i][2]
        r_squared_2 = bottom_x ** 2 + bottom_y ** 2
        bottom_camera_coordinates[i][0] = bottom_x * (1 + camera.k1 * r_squared_2 + camera.k2 * r_squared_2 ** 2 + camera.k3 * r_squared_2 ** 3) + 2 * camera.p1 * bottom_x * bottom_y + camera.p2 * (r_squared_2 + 2 * bottom_x ** 2)
        bottom_camera_coordinates[i][1] = bottom_y * (1 + camera.k1 * r_squared_2 + camera.k2 * r_squared_2 ** 2 + camera.k3 * r_squared_2 ** 3) + 2 * camera.p2 * bottom_x * bottom_y + camera.p1 * (r_squared_2 + 2 * bottom_y ** 2)
        bottom_camera_coordinates[i][2] = 1
        
        top_x = top_camera_coordinates[i][0] / top_camera_coordinates[i][2]
        top_y = top_camera_coordinates[i][1] / top_camera_coordinates[i][2]
        r_squared_2 = top_x ** 2 + top_y ** 2
        top_camera_coordinates[i][0] = top_x * (1 + camera.k1 * r_squared_2 + camera.k2 * r_squared_2 ** 2 + camera.k3 * r_squared_2 ** 3) + 2 * camera.p1 * top_x * top_y + camera.p2 * (r_squared_2 + 2 * top_x ** 2)
        top_camera_coordinates[i][1] = top_y * (1 + camera.k1 * r_squared_2 + camera.k2 * r_squared_2 ** 2 + camera.k3 * r_squared_2 ** 3) + 2 * camera.p2 * top_x * top_y + camera.p1 * (r_squared_2 + 2 * top_y ** 2)
        top_camera_coordinates[i][2] = 1

        bottom_image_coordinates[i] = np.matmul(camera.camera_matrix, bottom_camera_coordinates[i])
        top_image_coordinates[i] = np.matmul(camera.camera_matrix, top_camera_coordinates[i])

    return bottom_image_coordinates, top_image_coordinates

def cut_image(bottom_image_coordinates, top_image_coordinates, image):
    cut_imgs = []
    for i in range(NUMBER_OF_SILOS):
        cut_imgs.append = image[int(top_image_coordinates[i][1]) : int(bottom_image_coordinates[i][1]), int(bottom_image_coordinates[i][0] - 20) : int(bottom_image_coordinates[i][0] + 20)]
    
    return cut_imgs

def detect_balls(cut_img, silos):
    # 画像を縦方向に3分割
    height, width = cut_img.shape[:2]
    region_height = height // 3

    # 各領域のHSV割合を保存するリスト
    red_ratios = []
    blue_ratios = []

    for silo in range(NUMBER_OF_SILOS):
        for i in range(SILO_CAPACITY):
            if(silo.ball[i] == None):
                # 各領域の範囲を計算
                start_y = i * region_height
                end_y = (i + 1) * region_height

                # 領域を切り取り
                region = cut_img[start_y:end_y, :]

                # BGR形式からHSV形式に変換
                region_hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)

                # 赤色および青色のHSV範囲（調整が必要な場合は変更してください）
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
                
                if red_ratios[i] > 0.4:
                    silo.balls[i] = "red"
                    print(f"Region {i + 1}: Dominant Color - Red")
                elif blue_ratios[i] > 0.4:
                    silo.balls[i] = "blue"
                    print(f"Region {i + 1}: Dominant Color - Blue")
                else:
                    silo.balls[i] = "None"
                    print(f"Region {i + 1}: No dominant color or not dominant enough")
                
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






# def main():
    # # カメラのキャプチャを開始
    # # cap = cv2.VideoCapture(0)

    # # カメラの外部パラメータ(ここは機体座標と、機体の姿勢から逐次的に求める必要がある)
    # camera = Camera(Coordinate(0, 0, 0, 0))
    # # theta = np.radians(45)  # 45度をラジアンに変換
    # # R = np.array([
    # #     [np.cos(theta), -np.sin(theta), 0],
    # #     [np.sin(theta), np.cos(theta), 0],
    # #     [0, 0, 1]
    # # ])

    # # # 回転ベクトルの計算
    # # camera.rvec, _ = cv2.Rodrigues(R)
    # camera.rmat = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], dtype=np.float32)
    # camera.tvec = np.array([[0], [10], [0]], dtype=np.float32)

    # # while True:
    # #     # カメラからフレームを取得
    # #     ret, frame = cap.read()
    # #     if not ret:
    # #         break

    # #     # サイロの座標
    # #     silo = Silo(Coordinate(4.0, 2.0, 0, 0))

    # #     # カメラ画像上のサイロの頂点の座標を取得
    # #     # project_point = [np.array([0, 0], dtype=np.float32)]
    # #     # cv2.projectPoints(cv2.Mat(np.array([silo.coordinate.x, silo.coordinate.y, silo.coordinate.z], dtype=np.float32)), rvec, tvec, camera.camera_matrix, 0, project_point)
    # #     bottom_image_coordinate, _ = cv2.projectPoints(cv2.Mat(np.array([silo.coordinate.x, silo.coordinate.y, silo.bottom_z], dtype=np.float32)), rvec, tvec, camera.camera_matrix, 0)
    # #     top_image_coordinate, _ = cv2.projectPoints(cv2.Mat(np.array([silo.coordinate.x, silo.coordinate.y, silo.top_z], dtype=np.float32)), rvec, tvec, camera.camera_matrix, 0)


    # #     # 画像を切り取る
    # #     # print(type(bottom_image_coordinate))
    # #     print(bottom_image_coordinate[0, 0])
    # #     print(top_image_coordinate[0, 0])
    # #     cut_img = cut_image(bottom_image_coordinate, top_image_coordinate, frame)

    # #     # 画像を表示
    # #     cv2.imshow('Silo Only', cut_img)
    # #     # cv2.imshow('Silo Only', frame)

    # #     # 'q' キーでループを終了
    # #     if cv2.waitKey(1) & 0xFF == ord('q'):
    # #         break

    # # # キャプチャをリリースし、ウィンドウを閉じる
    # # cap.release()
    # # cv2.destroyAllWindows()






    # frame = cv2.imread('./c922/4_1.jpg')

    # # サイロの座標
    # silo = Silo(Coordinate(4000, 1000, 0, 0))

    # # カメラ画像上のサイロの頂点の座標を取得
    # bottom_camera_coordinate, top_camera_coordinate = world_to_camera_coordinate(silo, camera)
    # # print(bottom_camera_coordinate)
    # # print(top_camera_coordinate)
    # bottom_image_coordinate, top_image_coordinate = camera_to_image_coordinate(bottom_camera_coordinate, top_camera_coordinate, camera)
    # # print(bottom_image_coordinate)
    # # print(top_image_coordinate)


    # # 画像を切り取る
    # # print(type(bottom_image_coordinate))
    # # cut_img = cut_image(bottom_image_coordinate, top_image_coordinate, frame)

    # # 画像を表示
    # # cv2.imshow('Silo Only', cut_img)
    # cv2.circle(frame, (int(bottom_image_coordinate[0][0]), int(bottom_image_coordinate[1][0])), 10, (0, 0, 255), thickness=cv2.FILLED)
    # cv2.circle(frame, (int(top_image_coordinate[0][0]), int(top_image_coordinate[1][0])), 10, (0, 0, 255), thickness=cv2.FILLED)
    # cv2.namedWindow('Silo Only', cv2.WINDOW_NORMAL)
    # cv2.imshow('Silo Only', frame)

    # cv2.waitKey(0)
    # cv2.destroyAllWindows()