from ultralytics import YOLO
import cv2
import numpy as np

#ELECOMモデル
model = YOLO("/Users/atsum/Downloads/best.pt")

results = model(0 , show=True) 
for i in enumerate(results):
    print(i)


class SiloObserver(Node):
    def __init__(self):
        super().__init__('silo_observer_node')
        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
          'target_frame', 'base_link').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(SilosStatus, 'silos_status', 1)

        # on_timer関数を0.1秒ごとに実行
        self.timer = self.create_timer(0.1, self.on_timer)
    
    def on_timer(self):
        # transformの取得に使用する変数にフレーム名を格納する。
        from_frame_rel = self.target_frame
        to_frame_rel = 'map'

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            
            # カメラの外部パラメータ(ここは機体座標と、機体の姿勢から逐次的に求める必要がある)
            quat = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
            self.camera.rmat = quaternion_to_rotation_matrix(quat) * np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=np.float32)
            self.camera.rmat = np.array([[0, -1, 0], 
                     [0, 0, -1], 
                     [1, 0, 0]], dtype=np.float32)
            self.camera.tvec = np.matmul(self.camera.rmat, np.array([[t.transform.translation.x], [t.transform.translation.y], [t.transform.translation.z]], dtype=np.float32)) + np.array([[0], [0], [0]], dtype=np.float32)# 機体中心からカメラの位置までのベクトル

            # サイロの座標を取得
            count = 0
            for i in ['a', 'b', 'c', 'd', 'e']:
                frame = 'silo_' + i
                t = self.tf_buffer.lookup_transform(
                    to_frame_rel,
                    frame,
                    rclpy.time.Time())
                
                self.silos[count] = Silo(Coordinate(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z, 0))
                count += 1
            
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        # 現在の状態を取得
        self.observe_silos()

        # 過去の状態を保存
        if self.past_states.qsize() > 10:
            self.past_states.get()
            self.past_states.put([self.silos[0].balls, self.silos[1].balls, self.silos[2].balls, self.silos[3].balls, self.silos[4].balls])
        elif self.past_states.qsize() <= 10:
            self.past_states.put([self.silos[0].balls, self.silos[1].balls, self.silos[2].balls, self.silos[3].balls, self.silos[4].balls]) 
        
        # 過去の状態で最も多い状態を取得
        past_states = self.past_states.queue
        for i in range(NUMBER_OF_SILOS):
            past_state = []
            for j in range(len(past_states)):
                past_state.append(past_states[j][i]) 
            counter = Counter(map(tuple, past_state))
            most_common_state = max(counter, key=counter.get)
            self.silos[i].balls = list(most_common_state)

        # 状態をpublish
        msg = SilosStatus()
        msg.a.red = self.silos[0].balls.count("red")
        msg.a.blue = self.silos[0].balls.count("blue")
        msg.a.purple = self.silos[0].balls.count("purple")
        msg.b.red = self.silos[1].balls.count("red")
        msg.b.blue = self.silos[1].balls.count("blue")
        msg.b.purple = self.silos[1].balls.count("purple")
        msg.c.red = self.silos[2].balls.count("red")
        msg.c.blue = self.silos[2].balls.count("blue")
        msg.c.purple = self.silos[2].balls.count("purple")
        msg.d.red = self.silos[3].balls.count("red")
        msg.d.blue = self.silos[3].balls.count("blue")
        msg.d.purple = self.silos[3].balls.count("purple")
        msg.e.red = self.silos[4].balls.count("red")
        msg.e.blue = self.silos[4].balls.count("blue")
        msg.e.purple = self.silos[4].balls.count("purple")

        self.publisher.publish(msg)

        return

    # サイロの状態を観測
    def observe_silos(self):
        # カメラのキャプチャを開始
        cap = cv2.VideoCapture(0)
        # カメラからフレームを取得
        ret, frame = cap.read()

        # カメラ画像上のサイロの頂点の座標を取得
        self.silos = world_to_camera_coordinate(self.silos, self.camera)
        self.silos = camera_to_image_coordinate(self.silos, self.camera)

        # 画像を切り取ってボール検出
        self.silos, cut_imgs = cut_image(self.silos, frame)

        self.silos = detect_balls(self.silos, cut_imgs)

        # カットした画像表示
        xx = 0
        for img in cut_imgs:
            cv2.namedWindow('cut'+str(xx), cv2.WINDOW_NORMAL)
            cv2.imshow('cut'+str(xx), img)
            xx += 1

        # 画像を表示
        for silo in self.silos:
            if silo.detection_flag == True:
                cv2.circle(frame, (int(silo.bottom_image_coordinate[0][0]), int(silo.bottom_image_coordinate[1][0])), 10, (0, 0, 255), thickness=cv2.FILLED)
                cv2.circle(frame, (int(silo.top_image_coordinate[0][0]), int(silo.top_image_coordinate[1][0])), 10, (0, 0, 255), thickness=cv2.FILLED)
        cv2.namedWindow('Detected', cv2.WINDOW_NORMAL)
        cv2.imshow('Detected', frame)

        if cv2.waitKey(1) != -1:
            cv2.destroyAllWindows()
            cap.release()
            rclpy.shutdown()
        #メモリを解放して終了するためのコマンド
        cap.release()
        

def main(args=None):
    rclpy.init(args=args)

    silo_observer = SiloObserver()

    try:
        rclpy.spin(silo_observer)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()