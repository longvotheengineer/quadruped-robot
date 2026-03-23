# CHƯƠNG 4. MÔ PHỎNG VÀ KẾT QUẢ

## 4.1. Giới thiệu

Chương này trình bày kết quả mô phỏng hệ thống robot bốn chân sử dụng nền tảng ROS 2 Humble kết hợp với trình mô phỏng Gazebo Classic. Mục tiêu của quá trình mô phỏng là xác minh tính đúng đắn của các thuật toán lập kế hoạch quỹ đạo, động học nghịch (Inverse Kinematics), và bộ điều khiển PD trước khi triển khai trên robot thực. Toàn bộ kết quả được trình bày thông qua 12 hình minh họa, bao gồm các phân tích về quỹ đạo bàn chân, biên dạng góc khớp, mô-men xoắn điều khiển, ổn định dáng đi, không gian làm việc, kiến trúc hệ thống, và lưu đồ giải thuật.

## 4.2. Kiến trúc hệ thống mô phỏng

### 4.2.1. Tổng quan kiến trúc (Hình 9)

Hình 9 trình bày kiến trúc tổng quan của hệ thống mô phỏng, được tổ chức theo kiến trúc ba tầng (three-layer architecture):

**Tầng người dùng (User Layer):** Người vận hành tương tác với hệ thống thông qua giao diện Terminal bằng lệnh ROS 2 `ros2 topic pub`, cho phép gửi các lệnh dáng đi (ví dụ: `FORWARD`, `TURN_RIGHT`, `HOMING`) đến topic `/gait_control`. Trạng thái robot được giám sát trực quan qua công cụ RViz2.

**Tầng middleware ROS 2 Humble:** Đây là tầng trung gian chứa toàn bộ logic điều khiển và giao tiếp. Tầng này bao gồm các node chính sau:

- *node_leg_controller*: Node tùy chỉnh (custom node) chứa bốn module phần mềm — Gait Generator, Quintic Planning, Inverse Kinematics, và PD Controller. Node này nhận lệnh từ topic `/gait_control` (kiểu `String`), tính toán quỹ đạo và mô-men xoắn, sau đó xuất bản lệnh effort lên topic `/leg_controller/commands` (kiểu `Float64MultiArray`).
- *robot_state_publisher*: Xuất bản các phép biến đổi TF (Transform Frame) từ mô hình URDF phục vụ hiển thị trên RViz2.
- *controller_manager (ros2_control)*: Quản lý và khởi chạy các controller phần cứng.
- *joint_state_broadcaster*: Đọc trạng thái khớp (vị trí và vận tốc) từ Gazebo và xuất bản lên topic `/joint_states` (kiểu `JointState`).
- *leg_controller (JointGroupEffortController)*: Nhận lệnh effort (mô-men xoắn) và ghi trực tiếp xuống hardware interface của Gazebo.

**Tầng mô phỏng Gazebo:** Bao gồm bốn thành phần — Physics Engine (ODE) thực hiện tính toán va chạm và động lực học, mô hình URDF mô tả robot với 12 khớp revolute, hardware interface `gazebo_ros2_control` làm cầu nối giữa ros2_control và Gazebo, và module 3D Rendering hiển thị môi trường mô phỏng.

Luồng dữ liệu trong hệ thống được phân thành hai hướng: luồng lệnh (command flow) từ người dùng xuống Gazebo theo chiều feedforward, và luồng phản hồi (feedback flow) từ Gazebo ngược lên bộ điều khiển PD theo chiều feedback, tạo thành vòng điều khiển kín.

### 4.2.2. Kiến trúc điều khiển phần mềm (Hình 10)

Hình 10 trình bày sơ đồ khối (block diagram) của kiến trúc điều khiển dạng vòng kín, phân thành ba lớp chức năng:

**Lớp lập kế hoạch quỹ đạo (Trajectory Planning Layer)** thực hiện bốn bước xử lý tuần tự. Đầu tiên, module Gait Generator (`gaitGenerator.py`) nhận lệnh di chuyển và xác định mẫu dáng đi (gait pattern) cùng với offset pha cho từng chân. Tiếp theo, module Quintic Polynomial Planner (`quinticPlanning.py`) tạo quỹ đạo mượt sử dụng đa thức bậc 5 với điều kiện biên bậc hai (vận tốc và gia tốc bằng 0 tại hai đầu). Sau đó, module Inverse Kinematics (`kinematics.py`) chuyển đổi tọa độ Descartes của bàn chân trong body frame thành các góc khớp θ₁, θ₂, θ₃ thông qua phương pháp IK hình học cho cơ cấu 3 bậc tự do (3-DOF).

**Lớp điều khiển thấp (Low-Level Control Layer)** bao gồm hai module. Module Angle Conversion (`serialPublish.py`) thực hiện chuyển đổi đơn vị từ độ sang radian, thêm offset θ₃ + 180° cho phù hợp với quy ước URDF, và xử lý wrap-around ±π để tránh bước nhảy 2π. Module PD Controller (`controllerSim.py`) thực hiện luật điều khiển:

```
τ = Kp · (θ_target − θ_actual) − Kd · ω_actual
```

trong đó Kp = 20.0 Nm/rad là hệ số tỷ lệ, Kd = 0.5 Nm·s/rad là hệ số vi phân, θ_target là vị trí khớp mong muốn, θ_actual là vị trí khớp thực tế đo được, và ω_actual là vận tốc góc thực tế. Mô-men xoắn đầu ra được kẹp (clamp) trong giới hạn effort theo URDF.

**Lớp Plant (Gazebo/Robot)** nhận 12 giá trị mô-men xoắn qua `JointGroupEffortController`, mô phỏng phản ứng cơ học của robot trong môi trường vật lý (trọng lực 9.81 m/s², ma sát, phản lực mặt đất), và trả về trạng thái khớp (vị trí, vận tốc) qua Joint State Broadcaster, hoàn thành vòng điều khiển kín.

### 4.2.3. Lưu đồ giải thuật vòng điều khiển chính (Hình 11)

Hình 11 trình bày lưu đồ giải thuật (flowchart) của vòng điều khiển chính trong node `node_leg_controller`. Giải thuật hoạt động theo cơ chế máy trạng thái (state machine) kết hợp với timer callback chu kỳ 3 ms.

Khi khởi tạo, node đăng ký (subscribe) vào topic `/gait_control` để nhận lệnh dáng đi từ người dùng. Timer callback được kích hoạt mỗi 3 ms và thực hiện logic điều khiển theo hai trạng thái:

- **Trạng thái INIT:** Khi nhận lệnh mới, hệ thống gọi hàm `control_init()` để tiền tính toán toàn bộ quỹ đạo cho bốn chân (bao gồm sinh quỹ đạo, IK, và dịch pha). Sau khi hoàn thành, trạng thái chuyển sang READY.
- **Trạng thái READY:** Hệ thống gọi hàm `control_tick()` mỗi chu kỳ timer. Nếu lệnh là ZERO (homing), hàm `_tick_homing()` được gọi để đưa robot về vị trí home sử dụng PD torque. Nếu lệnh là dáng đi (FORWARD, TURN, ...), hàm `_tick_gait()` đọc góc khớp tại frame hiện tại, gọi `publish_simulation()` để chuyển đổi đơn vị và tính PD torque, sau đó tăng chỉ số frame.

Quá trình publish bao gồm bốn bước tuần tự: (1) chuyển đổi góc từ độ sang radian cho 12 khớp, (2) xử lý wrap-around ±π để tránh bước nhảy 2π, (3) tính mô-men xoắn PD theo công thức τ = Kp·e − Kd·ω, và (4) kẹp mô-men trong giới hạn URDF. Khi frame vượt quá tổng số waypoint, chỉ số frame được reset về 0 và bộ đếm bước (complete_step) tăng lên 1. Nếu đã hoàn thành đủ số bước yêu cầu, robot giữ vị trí (hold pose).

### 4.2.4. Lưu đồ giải thuật tạo dáng đi (Hình 12)

Hình 12 trình bày lưu đồ giải thuật tạo dáng đi, mô tả chi tiết quá trình từ khi nhận lệnh đến khi xuất bản lệnh điều khiển. Giải thuật được chia thành các bước chính sau:

**Bước phân loại lệnh:** Dựa trên trường `cmd` của lệnh nhận được, hệ thống phân nhánh xử lý. Nếu `cmd = ZERO`, hàm `generate_home()` tạo quỹ đạo homing cho bốn chân. Nếu `cmd` thuộc nhóm dáng đi (FORWARD, TURN_RIGHT, ...), hệ thống chọn bộ tham số tương ứng (PARAMS_GAIT_TROT hoặc PARAMS_GAIT_TURN).

**Bước 1 — Xác định điểm đầu/cuối:** Với mỗi chân (lặp FOR qua 4 chân), hệ thống xác định tọa độ điểm bắt đầu (pos_A) và kết thúc (pos_D) dựa trên tham số x_center, y_val, stride_length và z_stance.

**Bước 2 — Lập kế hoạch quỹ đạo:** Tùy thuộc vào cờ CONTROL_VELOCITY, hệ thống chọn một trong hai phương pháp: đa thức bậc 5 (quintic polynomial) với T_swing = 30, T_stance = 310 khi cờ TRUE, hoặc sóng sin (sine-wave) khi cờ FALSE. Cả hai phương pháp đều tạo mảng waypoint[T_total, 3] chứa tọa độ [x, y, z] cho mỗi waypoint.

**Bước 3 — Inverse Kinematics:** Với mỗi waypoint, thuật toán IK hình học 3-DOF chuyển đổi tọa độ Descartes thành ba góc khớp θ₁, θ₂, θ₃ (đơn vị: độ). Quá trình này bao gồm phép biến đổi tọa độ từ body frame sang leg-local frame.

**Bước 4 — Dịch pha (Phase Shift):** Mảng góc khớp được dịch vòng (circular shift) theo hệ số pha tương ứng với từng chân trong dáng đi Trot: LF = 0, LB = 0.5, RF = 0.5, RB = 0. Phép dịch pha đảm bảo hai cặp chân chéo nhau luân phiên swing.

Sau khi hoàn thành vòng lặp cho 4 chân, dữ liệu gait_angle_data[4][T_total, 3] được ghép lại và sẵn sàng cho hàm `control_tick()` thực thi frame-by-frame.

## 4.3. Phân tích quỹ đạo bàn chân

### 4.3.1. Quỹ đạo trong mặt phẳng 2D (Hình 1)

Hình 1 trình bày quỹ đạo bàn chân trong mặt phẳng XZ (mặt phẳng sagittal), so sánh hai phương pháp lập kế hoạch: sóng sin (sine-wave) và đa thức bậc 5 (quintic polynomial).

Quỹ đạo bàn chân có dạng hình chữ D, bao gồm hai pha: pha swing (nâng chân) trong đó bàn chân di chuyển theo đường cong từ điểm A đến điểm D với độ nâng `h = 40 mm` so với mặt đất, và pha stance (chạm đất) trong đó bàn chân trượt ngược trên mặt đất tạo lực đẩy cho robot.

Phương pháp sóng sin sử dụng hàm sin để tạo biên dạng nâng chân. Ưu điểm là đơn giản trong triển khai, tuy nhiên không kiểm soát được vận tốc và gia tốc tại các điểm chuyển pha. Phương pháp đa thức bậc 5 sử dụng hàm s(τ) = 10τ³ − 15τ⁴ + 6τ⁵ với τ ∈ [0, 1], đảm bảo các điều kiện biên: s(0) = 0, s(1) = 1, s'(0) = s'(1) = 0, s''(0) = s''(1) = 0. Nhờ đó, quỹ đạo có tính liên tục bậc hai (C²), giảm thiểu rung lắc cơ khí tại các điểm chuyển pha.

### 4.3.2. Quỹ đạo trong không gian 3D (Hình 2)

Hình 2 hiển thị quỹ đạo 3D của cả bốn chân trong hệ tọa độ thân robot (body frame), với kích thước thân L × W = 120 × 90 mm. Bốn quỹ đạo hình chữ D được phân bố tại bốn góc của khung thân, mỗi chân có offset pha tương ứng với dáng đi Trot. Dáng đi này có đặc điểm hai chân chéo nhau cùng thực hiện pha swing (LF+RB và LB+RF luân phiên), đảm bảo tại mọi thời điểm luôn có ít nhất hai chân chạm đất.

### 4.3.3. Biên dạng vận tốc và gia tốc (Hình 3)

Hình 3 so sánh biên dạng vận tốc và gia tốc của bàn chân theo hai trục X và Z giữa hai phương pháp, gồm bốn subplot:

Subplot (a) và (b) thể hiện vận tốc theo trục X (dx/dt) và trục Z (dz/dt). Kết quả cho thấy phương pháp đa thức bậc 5 có vận tốc bằng 0 tại cả hai đầu pha swing, trong khi phương pháp sóng sin có bước nhảy vận tốc tại ranh giới chuyển pha swing-stance.

Subplot (c) và (d) thể hiện gia tốc theo trục X (d²x/dt²) và trục Z (d²z/dt²). Sự khác biệt rõ rệt nhất ở biên dạng gia tốc: phương pháp sóng sin tạo các đỉnh gia tốc đột ngột (gián đoạn), trong khi đa thức bậc 5 cho biên dạng gia tốc liên tục. Kết quả này chứng minh rằng phương pháp đa thức bậc 5 ưu việt hơn về mặt mượt mà động học, giảm thiểu rung lắc và lực va đập tại các khớp.

## 4.4. Phân tích góc khớp và mô-men xoắn

### 4.4.1. Biến thiên góc khớp (Hình 4)

Hình 4 trình bày ba góc khớp đầu ra của thuật toán IK (θ₁, θ₂, θ₃) theo đơn vị độ cho cả bốn chân trong một chu kỳ dáng đi đầy đủ gồm T_total = 340 waypoints (T_swing = 30, T_stance = 310).

Góc θ₁ (khớp hip/coxa) điều khiển chuyển động abduction/adduction, dao động trong biên độ nhỏ (±2°) do robot di chuyển chủ yếu trong mặt phẳng sagittal. Hai chân bên trái có θ₁ < 0, hai chân bên phải có θ₁ > 0, phản ánh tính đối xứng cơ cấu.

Góc θ₂ (khớp femur) có biên độ lớn nhất, liên quan trực tiếp đến chiều dài bước (stride length). Do cấu hình hình học ngược nhau giữa các cặp chân (left-front/right-behind và left-behind/right-front), θ₂ có giá trị dương (~200°) cho một cặp và âm (~−200°) cho cặp còn lại.

Góc θ₃ (khớp tibia/gối) thay đổi rõ rệt trong pha swing khi bàn chân nâng lên để tránh va chạm mặt đất, với biên độ thay đổi khoảng 50° so với vị trí stance. Hiệu ứng offset pha của dáng đi Trot được quan sát rõ: cặp LF+RB có pha swing tại waypoint 0–30, cặp LB+RF có pha swing tại waypoint 170–200 (lệch 50% chu kỳ).

### 4.4.2. Ước lượng mô-men xoắn bộ điều khiển PD (Hình 5)

Hình 5 trình bày kết quả ước lượng mô-men xoắn cho chân Left-Front (chân tiêu biểu). Mô hình sử dụng luật điều khiển PD kết hợp bù trọng lực (gravity compensation):

```
τ_total = Kp · (θ_target − θ_actual) − Kd · ω_actual + τ_gravity
```

trong đó τ_gravity = m·g·l_cg·cos(θ) là mô-men trọng lực tại mỗi khớp, với m là khối lượng link, g = 9.81 m/s² là gia tốc trọng trường, và l_cg là khoảng cách đến trọng tâm link. Vị trí khớp thực tế (θ_actual) được mô hình hóa như phiên bản lọc thông thấp (low-pass filter) của vị trí mong muốn, mô phỏng độ trễ bám theo (tracking delay) đặc trưng của servo.

Kết quả phân tích cho ba khớp:

- **Khớp 1 (Hip):** Mô-men gần bằng 0 Nm trong suốt chu kỳ do chuyển động abduction tối thiểu.
- **Khớp 2 (Femur):** Mô-men lớn nhất đạt khoảng 28 Nm tại thời điểm bắt đầu pha swing, khi chân phải tăng tốc nhanh để nâng lên. Giá trị này nằm trong giới hạn effort của URDF (±29.3 Nm), xác nhận servo đủ công suất.
- **Khớp 3 (Tibia):** Mô-men dao động trong khoảng 5–7 Nm, nằm an toàn trong giới hạn (±19.2 Nm).

Các đường gạch đỏ trên hình biểu thị giới hạn effort theo URDF, vùng tô xanh nhạt là vùng hoạt động an toàn (safe zone). Toàn bộ mô-men xoắn đều nằm trong vùng an toàn, xác nhận khả năng vận hành của bộ truyền động.

## 4.5. Phân tích dáng đi và ổn định

### 4.5.1. Giản đồ thời gian dáng đi Trot (Hình 6)

Hình 6 hiển thị giản đồ thời gian (timing diagram) của dáng đi Trot cho bốn chân, trong đó thanh màu xanh biểu thị pha swing và vùng trắng biểu thị pha stance.

Dáng đi Trot có tỷ lệ duty cycle β = T_swing / T_total = 30/340 ≈ 8.8%. Điều này có nghĩa mỗi chân chỉ ở trạng thái swing khoảng 8.8% thời gian và ở trạng thái stance 91.2% thời gian. Hai cặp chân chéo nhau (LF+RB và LB+RF) luân phiên thực hiện pha swing với độ lệch pha 50% chu kỳ, đảm bảo tại mọi thời điểm luôn có ít nhất hai chân chạm đất.

### 4.5.2. Đa giác đỡ và ổn định tĩnh (Hình 7)

Hình 7 phân tích ổn định tĩnh bằng phương pháp đa giác đỡ (Support Polygon) tại hai thời điểm đặc trưng trong chu kỳ dáng đi.

Tại thời điểm t = 0 (pha swing của cặp LF+RB), chỉ có hai chân LB và RF chạm đất. Đa giác đỡ suy biến thành đoạn thẳng nối hai bàn chân. Hình chiếu trọng tâm (Center of Mass — CoM) của robot xuống mặt phẳng đỡ nằm gần đường thẳng này, cho thấy ổn định ở mức biên (marginal stability).

Tại thời điểm t = T/4 (pha stance đầy đủ), cả bốn chân đều chạm đất. Đa giác đỡ là hình chữ nhật bao quanh robot với diện tích lớn. Hình chiếu CoM nằm hoàn toàn bên trong đa giác đỡ, cho thấy ổn định tĩnh cao.

Tiêu chí ổn định được sử dụng: robot ổn định tĩnh khi và chỉ khi hình chiếu trọng tâm nằm trong bao lồi (convex hull) của các điểm tiếp xúc chân-đất. Kết quả xác nhận dáng đi Trot với các tham số đã chọn duy trì ổn định trong suốt chu kỳ.

## 4.6. Không gian làm việc (Hình 8)

Hình 8 trình bày không gian làm việc (workspace) của một chân robot 3-DOF, tính bằng phương pháp quét động học thuận (Forward Kinematics sweep).

Phương pháp tính toán bao gồm: quét toàn bộ miền góc khớp θ₁ ∈ [−30°, 30°], θ₂ ∈ [−90°, 90°], θ₃ ∈ [−90°, 90°] với khoảng 90.000 tổ hợp góc. Tại mỗi tổ hợp, tọa độ đầu chân trong hệ tọa độ chân (leg-local frame) được tính bằng phương trình FK và vẽ thành đám mây điểm 3D.

Kết quả cho thấy workspace có dạng **vòng cung** (annular sector), đặc trưng cho cơ cấu serial 3-DOF. Vùng đạt được rộng nhất trong mặt phẳng Y-Z (sagittal), trong khi chiều sâu theo phương X bị giới hạn bởi biên độ khớp hip (θ₁). Tầm với tối đa của chân là l₂ + l₃ = 80 + 80 = 160 mm, tương ứng với trường hợp xương đùi và xương ống chân duỗi thẳng hoàn toàn.

## 4.7. Tổng hợp tham số hệ thống

Bảng 4.1 tổng hợp các tham số chính của hệ thống mô phỏng.

**Bảng 4.1.** Tham số hệ thống robot bốn chân

| Tham số | Ký hiệu | Giá trị | Đơn vị |
|---------|----------|---------|--------|
| Chiều dài thân robot | L | 120 | mm |
| Chiều rộng thân robot | W | 90 | mm |
| Chiều dài khớp coxa | l₁ | 20 | mm |
| Chiều dài xương đùi | l₂ | 80 | mm |
| Chiều dài xương ống chân | l₃ | 80 | mm |
| Hệ số tỷ lệ PD | Kp | 20.0 | Nm/rad |
| Hệ số vi phân PD | Kd | 0.5 | Nm·s/rad |
| Giới hạn effort khớp hip | τ_max (hip) | 29.3 | Nm |
| Giới hạn effort khớp tibia | τ_max (tibia) | 19.2 | Nm |
| Số waypoint pha swing | T_swing | 30 | waypoints |
| Số waypoint pha stance | T_stance | 310 | waypoints |
| Độ cao nâng chân | h | 40 | mm |
| Chu kỳ timer điều khiển | T_timer | 3 | ms |
| Tốc độ cập nhật ros2_control | f_ctrl | 1000 | Hz |

## 4.8. Nhận xét và đánh giá

Các kết quả mô phỏng cho phép rút ra một số nhận xét sau:

Thứ nhất, phương pháp lập kế hoạch quỹ đạo bằng đa thức bậc 5 cho kết quả vượt trội so với phương pháp sóng sin về mặt mượt mà động học. Biên dạng vận tốc và gia tốc liên tục (C²) tại các điểm chuyển pha giúp giảm thiểu lực va đập và rung lắc tại các khớp, kéo dài tuổi thọ cơ cấu truyền động.

Thứ hai, bộ điều khiển PD với các tham số Kp = 20.0 Nm/rad và Kd = 0.5 Nm·s/rad tạo ra mô-men xoắn nằm trong giới hạn cho phép của servo tại cả ba khớp. Mô-men lớn nhất xuất hiện tại khớp femur (~28 Nm) vẫn thấp hơn giới hạn URDF (29.3 Nm), xác nhận bộ truyền động đủ công suất cho dáng đi Trot.

Thứ ba, dáng đi Trot với duty cycle β ≈ 8.8% duy trì ổn định tĩnh trong suốt chu kỳ. Phân tích đa giác đỡ cho thấy biên ổn định nhỏ nhất xảy ra tại thời điểm chuyển pha khi chỉ có hai chân chạm đất, nhưng vẫn đảm bảo hình chiếu trọng tâm nằm trong vùng đỡ.

Thứ tư, không gian làm việc dạng vòng cung với tầm với tối đa 160 mm đủ lớn để thực hiện các bước đi trong phạm vi thiết kế. Kiến trúc phần mềm phân tầng (trajectory planning — low-level control — plant) cho phép mô phỏng trước khi triển khai trên phần cứng thực với thay đổi tối thiểu.
