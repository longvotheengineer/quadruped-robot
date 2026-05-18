with open('/home/an/quadruped-robot/Software/Middleware/robot_ros2/src/thesis/chapter/ch2_lythuyet.tex', 'r') as f:
    content = f.read()

new_section = r"""\section{Thiết kế bộ điều khiển}

Để robot có thể di chuyển ổn định và phản hồi chính xác với các nhiễu loạn từ môi trường, một kiến trúc điều khiển tổng thể (Overall Control Architecture) được thiết kế kết hợp giữa khối quy hoạch dáng đi (Gait Mapping) và khối điều khiển tư thế (Attitude Adjustment), như được minh họa khái quát trong Hình~\ref{fig:overall_control_architecture}. 

Trong kiến trúc này, quỹ đạo bàn chân lý thuyết (Feet trajectories) được tạo ra từ khối quy hoạch dáng đi và truyền đến bộ điều khiển tư thế PID. Cùng lúc đó, cảm biến IMU gắn trên thân robot liên tục phản hồi góc nghiêng thực tế (Roll, Pitch). Bộ điều khiển PID sẽ tính toán lượng bù và điều chỉnh lại quỹ đạo bàn chân này để chống lại độ nghiêng thân. Tọa độ bàn chân sau khi được hiệu chỉnh sẽ được đưa qua khối Động học nghịch (IK Solution) để giải ra các góc khớp mục tiêu (Joint angles) điều khiển trực tiếp các động cơ servo.

Sơ đồ khối vòng lặp kín chi tiết của hệ thống điều khiển cân bằng áp dụng trong đồ án được trình bày trong Hình~\ref{fig:applied_pid_simulink}.

\begin{figure}[H]
    \centering
    \includegraphics[width=\textwidth]{img_matlab/fig-PID.jpg}
    \caption{Sơ đồ khối vòng lặp kín điều khiển cân bằng hệ thống áp dụng trong đồ án}
    \label{fig:applied_pid_simulink}
\end{figure}

\begin{figure}[H]
    \centering
    \includegraphics[width=0.9\textwidth]{architecture/overall_control_architecture.png}
    \caption{Sơ đồ tổng thể kiến trúc điều khiển kết hợp quy hoạch dáng đi và điều chỉnh tư thế}
    \label{fig:overall_control_architecture}
\end{figure}

\subsection{Kiến trúc điều khiển chi tiết}
Hệ thống điều khiển tổng thể của robot hoạt động với tần số vòng lặp 333~Hz ($\Delta T = 3$~ms), được thiết kế theo cấu trúc phân cấp chặt chẽ. Kiến trúc này tiếp nhận các lệnh cấp cao thông qua một máy trạng thái (Command Manager) và điều phối luồng dữ liệu xuống hai thành phần xử lý cốt lõi: Bộ quy hoạch chuyển động (Locomotion Planner) và Bộ điều khiển cân bằng (Balance Controller), như được minh họa trong Hình \ref{fig28}.

\begin{figure}[H]
    \centering
    \includegraphics[width=\textwidth]{architecture/locomotion_balance_controller.png}
    \caption{Tổng quan kiến trúc điều khiển của hệ thống (Locomotion Planner \& Balance Controller)}
    \label{fig28}
\end{figure}

Đặc điểm nổi bật của kiến trúc này là sự xuất hiện của công tắc chuyển mạch điều chỉnh cân bằng (Balance Output Switch), cho phép rẽ nhánh luồng dữ liệu bù trừ tư thế dựa trên hai chế độ hoạt động chính của robot:
\begin{itemize}
    \item \textbf{Chế độ tĩnh (HOME) -- Bù trong không gian khớp (Joint Space):} 
    Khi robot đứng yên tại chỗ, khối Locomotion Planner duy trì tọa độ đứng danh định. Trong khi đó, dữ liệu Roll và Pitch từ cảm biến IMU liên tục được đưa qua bộ điều khiển PID để tính toán lượng bù góc khớp. Lượng bù này ($\Delta\theta_3$) được cộng trực tiếp vào góc khớp gối ở khâu cuối cùng (sau khối Động học nghịch IK), ngay trước khi gửi lệnh xuống động cơ. Cấu trúc này đảm bảo phản ứng cực kỳ nhanh nhạy để giữ thăng bằng chống lại nhiễu loạn ngoại lực.
    
    \item \textbf{Chế độ động (GAIT) -- Bù trong không gian làm việc (Task Space):} 
    Khi robot thực hiện dáng đi, bộ lập lịch pha (Gait Phase Scheduler) tạo ra quỹ đạo bàn chân 3D. Đồng thời, lượng hiệu chỉnh từ PID ($\Delta\mathbf{p}$) được định tuyến để can thiệp vào không gian làm việc (Task Space). Tọa độ bàn chân được cộng bù trực tiếp trong bộ Gait Attitude Compensator, sau đó mới đi qua khối Động học nghịch (IK). Phương pháp này giúp duy trì sải bước chuẩn xác và không làm biến dạng biên độ bước đi của robot.
\end{itemize}

\subsection{Cơ chế áp dụng tín hiệu điều chỉnh tư thế}
Việc điều chỉnh tư thế được thực hiện theo thời gian thực thông qua quá trình đo đạc góc nghiêng từ cảm biến IMU. Đầu ra của các bộ điều khiển PID là các tín hiệu bù hiệu chỉnh $u_{roll}$ và $u_{pitch}$. Sự khác biệt cốt lõi giữa hai chế độ hoạt động nằm ở \textit{không gian} mà tín hiệu hiệu chỉnh được ánh xạ:

\textbf{1. Áp dụng hiệu chỉnh trong không gian khớp (Joint Space) ở chế độ HOME:} \\
Khi robot ở trạng thái tĩnh, tín hiệu $u_{roll}, u_{pitch}$ từ bộ PID được phân bổ thành offset cho từng chân, với giới hạn bão hòa $|\Delta\theta_3| \leq \theta_{sat}$:
\begin{equation}
\begin{cases} 
\Delta\theta_{3,LF} = \mathrm{sat}\big(- (K_{home} u_{roll} - K_{home} u_{pitch}),\; \theta_{sat}\big) \\
\Delta\theta_{3,LB} = \mathrm{sat}\big(- (K_{home} u_{roll} + K_{home} u_{pitch}),\; \theta_{sat}\big) \\
\Delta\theta_{3,RF} = \mathrm{sat}\big(- (K_{home} u_{roll} + K_{home} u_{pitch}),\; \theta_{sat}\big) \\
\Delta\theta_{3,RB} = \mathrm{sat}\big(- (K_{home} u_{roll} - K_{home} u_{pitch}),\; \theta_{sat}\big) 
\end{cases}
\end{equation}
Lượng bù này được cộng trực tiếp làm offset cho góc khớp gối ($\theta_3$). Do cấu trúc động học chuỗi hở, việc thay đổi góc khớp gối $\theta_3$ sẽ thay đổi vị trí bàn chân, tạo ra các lực phản lực để robot lấy lại cân bằng tức thời mà không cần qua khối giải động học IK phức tạp.

\textbf{2. Áp dụng hiệu chỉnh trong không gian làm việc (Task Space) ở chế độ GAIT:} \\
Khi robot di chuyển, quỹ đạo bước chân phải được bảo toàn nghiêm ngặt. Ma trận xoay nghịch đảo $R = R_y(u_{pitch}) R_x(u_{roll})$ được tính toán dựa trên lượng bù từ PID. Ma trận này áp dụng lên vectơ vị trí chân $\mathbf{p}_{raw}$ sinh ra từ bộ tạo quỹ đạo. Hệ thống chủ động \textbf{chỉ áp dụng lượng bù cho duy nhất tọa độ dọc $Z$} của bàn chân, và giữ nguyên giá trị trục $X, Y$:
\begin{equation}
\mathbf{p}_{adj} = R \cdot \mathbf{p}_{raw}, \quad \text{với} \quad x_{adj} = x_{raw}, \quad y_{adj} = y_{raw}
\end{equation}

Việc giữ nguyên sự thay đổi trên trục $X, Y$ là cực kỳ quan trọng để chiều dài bước đi (stride length) không bị biến dạng. Tọa độ Đề-các mới $[x_{adj}, y_{adj}, z_{adj}]$ sau đó được đưa qua bộ động học nghịch (IK) để tính góc quay mới cho các khớp. Quá trình này được minh họa trực quan trong Hình \ref{fig29}.

\begin{figure}[H]
    \centering
    \includegraphics[width=0.9\textwidth]{kinematics/attitude_adjustment.png}
    \caption{Minh họa quá trình điều chỉnh tư thế thân robot: từ $\mathbf{p}_{raw}$ (trước bù) sang $\mathbf{p}_{adj}$ (sau bù), chỉ thay đổi $z$, giữ nguyên $x, y$}
    \label{fig29}
\end{figure}

\subsection{Thuật toán điều khiển PID và các khâu bảo vệ}
Trong cả hai chế độ (HOME và GAIT), thuật toán cốt lõi xử lý sai lệch góc nghiêng là bộ điều khiển PID kinh điển, được triển khai độc lập cho trục Roll và Pitch. Tín hiệu đầu vào là sai số $e(t) = 0 - \theta_{meas}(t)$, trong đó $\theta_{meas}$ là góc đo từ IMU đã qua bộ lọc trung bình trượt hàm mũ (EMA) để giảm thiểu nhiễu cơ học.

Luật điều khiển PID được biểu diễn dưới dạng rời rạc:
\begin{equation}
    u_{PID}(k) = K_P e(k) + K_I \sum_{i=0}^{k} e(i) \Delta T + K_D \frac{e(k) - e(k-1)}{\Delta T}
\end{equation}

Để đảm bảo an toàn tuyệt đối cho phần cứng robot khi vận hành, đầu ra của bộ PID phải đi qua các khâu bảo vệ thiết yếu trước khi truyền đến cơ cấu chấp hành:
\begin{itemize}
    \item \textbf{Khâu triệt vùng chết (Deadband):} Loại bỏ các sai lệch quá nhỏ dao động quanh gốc $0$, tránh hiện tượng chattering (động cơ nhích liên tục gây ồn và nóng) khi robot đang cân bằng.
    \item \textbf{Bão hòa tín hiệu (Saturation) và Lọc thông thấp (LPF):} Tín hiệu điều khiển $u_{PID}$ được giới hạn trong ngưỡng an toàn $[-u_{max}, u_{max}]$ để tránh vọt lố biên độ quá lớn. Sau đó, nó đi qua một bộ lọc thông thấp (Low-Pass Filter) để vuốt mượt, giúp các khớp phản hồi mềm mại thay vì thay đổi đột ngột.
    \item \textbf{Chống tích lũy (Anti-windup):} Khi tín hiệu đầu ra chạm ngưỡng bão hòa, cơ chế anti-windup sẽ khóa hoặc giảm thành phần tích phân $I$, ngăn chặn sự tích lũy sai số dư thừa, giúp hệ thống không bị dao động hoặc phản hồi trễ khi ngoại lực biến mất.
\end{itemize}

Sự phân tách linh hoạt giữa không gian khớp và không gian làm việc, kết hợp với các cơ chế bảo vệ tín hiệu của thuật toán PID, là yếu tố then chốt giúp hệ thống cân bằng vận hành khoa học và bám sát lý thuyết động lực học.
"""

index = content.find(r'\section{Thiết kế bộ điều khiển}')
if index != -1:
    new_content = content[:index] + new_section
    with open('/home/an/quadruped-robot/Software/Middleware/robot_ros2/src/thesis/chapter/ch2_lythuyet.tex', 'w') as f:
        f.write(new_content)
    print("Done")
else:
    print("Pattern not found")
