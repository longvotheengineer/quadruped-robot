import os
import re

base_dir = "chapter"

def read_file(filename):
    with open(os.path.join(base_dir, filename), "r", encoding="utf-8") as f:
        return f.read()

def write_file(filename, content):
    with open(os.path.join(base_dir, filename), "w", encoding="utf-8") as f:
        f.write(content)

c1 = read_file("chapter1-gioithieu.tex")
c2 = read_file("chapter2-tongquan.tex")
c3 = read_file("chapter3-phantichcocau.tex")
c4 = read_file("chapter4-donghoc.tex")
c5 = read_file("chapter5-thietkevaxaydunghethong.tex")
c6 = read_file("chapter6-mophong.tex")
c7 = read_file("chapter7-ketluan.tex")

def extract_section(text, sec_title, end_markers):
    start_idx = text.find(f"\\section{{{sec_title}}}")
    if start_idx == -1:
        match = re.search(r"\\section\{" + sec_title + r".*?\}", text)
        if match:
            start_idx = match.start()
        else:
            return ""
            
    end_idx = len(text)
    for marker in end_markers:
        idx = text.find(marker, start_idx + 10)
        if idx != -1 and idx < end_idx:
            end_idx = idx
            
    return text[start_idx:end_idx].strip() + "\n\n"

# --- CH 1 ---
ch1_content = "% !TeX root = ../thesis.tex\n\\chapter{GIỚI THIỆU}\n\n"

# 1.1 Tổng quan -> "Đặt vấn đề và lý do chọn đề tài"
sec1 = extract_section(c1, "Đặt vấn đề và lý do chọn đề tài", ["\\section{Mục tiêu", "\\section{Phạm vi"])
sec1 = sec1.replace("\\section{Đặt vấn đề và lý do chọn đề tài}", "\\section{Tổng quan}")
ch1_content += sec1

# 1.2 Tình hình nghiên cứu
ch1_content += "\\section{Tình hình nghiên cứu trong và ngoài nước}\n\n"
sec2_1 = extract_section(c2, "Lịch sử phát triển robot bốn chân", ["\\section{Các nền tảng"]).replace("\\section{Lịch sử phát triển robot bốn chân}", "\\subsection{Lịch sử phát triển robot bốn chân}")
sec2_2 = extract_section(c2, "Các nền tảng quadruped robot tiêu biểu", ["\\section{Lý thuyết"]).replace("\\section{Các nền tảng quadruped robot tiêu biểu}", "\\subsection{Các nền tảng quadruped robot tiêu biểu}")
ch1_content += sec2_1 + sec2_2

# 1.3 Nhiệm vụ luận văn -> "Mục tiêu nghiên cứu" & "Phạm vi đề tài" & "Bố cục đồ án"
ch1_content += "\\section{Nhiệm vụ luận văn}\n\n"
sec3_1 = extract_section(c1, "Mục tiêu nghiên cứu", ["\\section{Phạm vi"]).replace("\\section{Mục tiêu nghiên cứu}", "\\subsection{Mục tiêu nghiên cứu}")
sec3_2 = extract_section(c1, "Phạm vi đề tài", ["\\section{Bố cục"]).replace("\\section{Phạm vi đề tài}", "\\subsection{Phạm vi đề tài}")
sec3_3 = extract_section(c1, "Bố cục đồ án", ["\\chapter{", "% END"]).replace("\\section{Bố cục đồ án}", "\\subsection{Bố cục đồ án}")
ch1_content += sec3_1 + sec3_2 + sec3_3

write_file("ch1_gioithieu.tex", ch1_content)

# --- CH 2 ---
ch2_content = "% !TeX root = ../thesis.tex\n\\chapter{LÝ THUYẾT}\n\n"
ch2_content += extract_section(c2, "Lý thuyết cơ bản về hình học và cân bằng", ["\\chapter{", "% END"])
c4_mod = re.sub(r"\\chapter\{.*?\}", "", c4, count=1).strip()
ch2_content += c4_mod
write_file("ch2_lythuyet.tex", ch2_content)

# --- CH 3 ---
ch3_content = "% !TeX root = ../thesis.tex\n\\chapter{THIẾT KẾ VÀ THỰC HIỆN PHẦN CỨNG}\n\n"
c3_mod = re.sub(r"\\chapter\{.*?\}", "", c3, count=1).strip()
ch3_content += c3_mod + "\n\n"
ch3_content += extract_section(c5, "Tổng quan về kiến trúc hệ thống", ["\\section{Các Giao thức"])
ch3_content += extract_section(c5, "Các Giao thức Truyền thông của Hệ thống", ["\\section{Kiến trúc phần mềm"])
write_file("ch3_thietkecung.tex", ch3_content)

# --- CH 4 ---
ch4_content = "% !TeX root = ../thesis.tex\n\\chapter{THIẾT KẾ VÀ THỰC HIỆN PHẦN MỀM}\n\n"
ch4_content += extract_section(c5, "Kiến trúc phần mềm ROS 2", ["\\section{Quy trình Homing"])
ch4_content += extract_section(c5, "Quy trình Homing và Calibrate", ["\\chapter{", "% END"])
write_file("ch4_thietkemem.tex", ch4_content)

# --- CH 5 ---
ch5_content = "% !TeX root = ../thesis.tex\n\\chapter{KẾT QUẢ THỰC HIỆN}\n\n"
c6_mod = re.sub(r"\\chapter\{.*?\}", "", c6, count=1).strip()
ch5_content += c6_mod
write_file("ch5_ketqua.tex", ch5_content)

# --- CH 6 ---
ch6_content = "% !TeX root = ../thesis.tex\n\\chapter{KẾT LUẬN VÀ HƯỚNG PHÁT TRIỂN}\n\n"
c7_mod = re.sub(r"\\chapter\{.*?\}", "", c7, count=1).strip()
sec_ketluan = extract_section(c7_mod, "Kết luận", ["\\section{Đóng góp"])
sec_donggop = extract_section(c7_mod, "Đóng góp của đề tài", ["\\section{Hạn chế}"]).replace("\\section{Đóng góp của đề tài}", "\\subsection{Đóng góp của đề tài}")
sec_hanche = extract_section(c7_mod, "Hạn chế", ["\\section{Hướng phát triển}"]).replace("\\section{Hạn chế}", "\\subsection{Hạn chế}")
sec_huongphattrien = extract_section(c7_mod, "Hướng phát triển", ["\\chapter{", "% END"])

ch6_content += sec_ketluan + sec_donggop + sec_hanche + sec_huongphattrien
write_file("ch6_ketluan.tex", ch6_content)

print("Files recreated successfully.")
