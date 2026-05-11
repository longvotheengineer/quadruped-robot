import re

with open('/home/an/quadruped-robot/Software/Middleware/robot_ros2/src/thesis/chapter/ch5_ketqua.tex', 'r', encoding='utf-8') as f:
    content = f.read()

# Extract HOME block
home_pattern = re.compile(r'\nHình \\ref\{fig:sim_home\}.*?\\end\{figure\}\n', re.DOTALL)
home_match = home_pattern.search(content)
home_text = home_match.group(0) if home_match else ""

# Extract GAIT block
gait_pattern = re.compile(r'\nHình \\ref\{fig:sim_trot_sequence\}.*?\\end\{figure\}\n', re.DOTALL)
gait_match = gait_pattern.search(content)
gait_text = gait_match.group(0) if gait_match else ""

if home_text and gait_text:
    # Delete from 5.3
    content = content.replace(home_text, '')
    content = content.replace(gait_text, '')

    # Insert to 5.2
    insert_target = "Tốc độ cập nhật bộ điều khiển ROS~2 được đặt ở 1000 Hz ($f_{ctrl} = 1000$ Hz), đồng bộ với bước thời gian vật lý của Gazebo. Chu kỳ timer của Gait Generator node là 3 ms, tương ứng với tần số xuất lệnh quỹ đạo 333 Hz."
    
    insertion = insert_target + "\n\n\\subsection{Kết quả mô phỏng vận động cơ bản}\n\nSau khi cấu hình xong môi trường, các thử nghiệm vận động cơ bản được tiến hành trên Gazebo để đánh giá khả năng di chuyển động học trước khi phân tích sâu vào hệ thống điều khiển.\n" + home_text + "\n" + gait_text + "\n"

    content = content.replace(insert_target, insertion)

    with open('/home/an/quadruped-robot/Software/Middleware/robot_ros2/src/thesis/chapter/ch5_ketqua.tex', 'w', encoding='utf-8') as f:
        f.write(content)
    print("Success")
else:
    print("Failed to find blocks")
