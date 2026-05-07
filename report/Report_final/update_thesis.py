import re

with open('thesis.tex', 'r', encoding='utf-8') as f:
    lines = f.readlines()

new_lines = []
skip = False

i = 0
while i < len(lines):
    line = lines[i]
    
    # 1. Skip TRANG XÁC NHẬN CÔNG TRÌNH ... to end of TRANG NHẬN XÉT CÁN BỘ PHẢN BIỆN
    if line.startswith('% 1. TRANG XÁC NHẬN CÔNG TRÌNH'):
        skip = True
        
    if skip and line.strip() == '% 4. NHIỆM VỤ KHÓA LUẬN TỐT NGHIỆP':
        # Stop skipping, but we also want to replace this section
        skip = False
        # remove the % ======================================================== above it
        if len(new_lines) > 0 and new_lines[-1].startswith('% ===='):
            new_lines.pop()

    # 2. Replace NHIỆM VỤ KHÓA LUẬN TỐT NGHIỆP
    if not skip and line.strip() == '% 4. NHIỆM VỤ KHÓA LUẬN TỐT NGHIỆP':
        new_lines.append('% ========================================================\n')
        new_lines.append('% NHIỆM VỤ KHÓA LUẬN TỐT NGHIỆP\n')
        new_lines.append('% ========================================================\n')
        new_lines.append('\\newpage\n')
        new_lines.append('\\thispagestyle{empty}\n')
        new_lines.append('\\begin{center}\n')
        new_lines.append('\\begin{longtable}{|p{0.95\\textwidth}|}\n')
        new_lines.append('\\hline\n')
        new_lines.append('\\vspace{0.1cm}\n')
        new_lines.append('\\begin{minipage}[t]{0.45\\textwidth}\n')
        new_lines.append('    \\centering\n')
        new_lines.append('    ĐẠI HỌC QUỐC GIA TP.HCM \\\\\n')
        new_lines.append('    TRƯỜNG ĐẠI HỌC BÁCH KHOA \\\\\n')
        new_lines.append('    \\textbf{KHOA ĐIỆN -- ĐIỆN TỬ}\n')
        new_lines.append('\\end{minipage}%\n')
        new_lines.append('\\hfill\n')
        new_lines.append('\\begin{minipage}[t]{0.45\\textwidth}\n')
        new_lines.append('    \\centering\n')
        new_lines.append('    CỘNG HÒA XÃ HỘI CHỦ NGHĨA VIỆT NAM \\\\\n')
        new_lines.append('    \\textbf{Độc Lập -- Tự Do -- Hạnh Phúc}\n')
        new_lines.append('\\end{minipage}\n')
        new_lines.append('\n')
        new_lines.append('\\vspace{1.5cm}\n')
        new_lines.append('\\begin{center}\n')
        new_lines.append('    {\\fontsize{14pt}{1.5}\\selectfont \\textbf{NHIỆM VỤ KHÓA LUẬN TỐT NGHIỆP}} \\\\[0.2cm]\n')
        new_lines.append('    \\textit{(Chú ý: sinh viên phải dán tờ này vào trang thứ nhất của bản thuyết minh)}\n')
        new_lines.append('\\end{center}\n')
        new_lines.append('\\vspace{0.5cm}\n')
        new_lines.append('\n')
        new_lines.append('\\textbf{HỌ VÀ TÊN:} \\dotfill \\hspace{1cm} \\textbf{MSSV:} \\dotfill \\\\\n')
        new_lines.append('\\textbf{NGÀNH:} \\dotfill \\hspace{1cm} \\textbf{LỚP:} \\dotfill \\\\[0.5cm]\n')
        new_lines.append('\n')
        new_lines.append('\\textbf{1. Đầu đề khóa luận:} \\\\\n')
        new_lines.append('Phân tích và thiết kế \\dots \\\\\n')
        new_lines.append('(\\textit{Analysis and design \\dots\\dots\\dots\\dots\\dots\\dots\\dots}) \\\\[0.5cm]\n')
        new_lines.append('\n')
        new_lines.append('\\textbf{2. Nhiệm vụ (yêu cầu về nội dung và số liệu ban đầu):} \\\\\n')
        new_lines.append('- \\dotfill \\\\\n')
        new_lines.append('- \\dotfill \\\\\n')
        new_lines.append('- \\dotfill \\\\\n')
        new_lines.append('- \\dotfill \\\\\n')
        new_lines.append('- \\dotfill \\\\[0.5cm]\n')
        new_lines.append('\n')
        new_lines.append('\\textbf{3. Ngày giao nhiệm vụ khóa luận:} DD / MM / YYYY \\\\[0.5cm]\n')
        new_lines.append('\\textbf{4. Ngày hoàn thành nhiệm vụ:} DD / MM / YYYY \\\\[0.5cm]\n')
        new_lines.append('\\textbf{5. Họ và tên người hướng dẫn:} \\hspace{2cm} \\textbf{Phần hướng dẫn:} \\\\\n')
        new_lines.append('- Nguyễn Văn A \\dotfill \\\\\n')
        new_lines.append('- Nguyễn Văn B \\dotfill \\\\[0.5cm]\n')
        new_lines.append('\n')
        new_lines.append('Nội dung và yêu cầu KLTN đã được thông qua Chủ trì ngành. \\\\[0.2cm]\n')
        new_lines.append('\\textit{Ngày \\dots\\dots tháng \\dots\\dots năm \\dots\\dots\\dots} \\\\[0.5cm]\n')
        new_lines.append('\n')
        new_lines.append('\\begin{minipage}[t]{0.45\\textwidth}\n')
        new_lines.append('    \\centering\n')
        new_lines.append('    \\textbf{CHỦ TRÌ NGÀNH} \\\\[0.2cm]\n')
        new_lines.append('    (Ký và ghi rõ họ tên)\n')
        new_lines.append('\\end{minipage}%\n')
        new_lines.append('\\hfill\n')
        new_lines.append('\\begin{minipage}[t]{0.45\\textwidth}\n')
        new_lines.append('    \\centering\n')
        new_lines.append('    \\textbf{GIẢNG VIÊN HƯỚNG DẪN CHÍNH} \\\\[0.2cm]\n')
        new_lines.append('    (Ký và ghi rõ họ tên)\n')
        new_lines.append('\\end{minipage}\n')
        new_lines.append('\n')
        new_lines.append('\\vspace{3cm}\n')
        new_lines.append('\n')
        new_lines.append('\\textbf{PHẦN DÀNH CHO KHOA} \\\\\n')
        new_lines.append('Người duyệt (chấm sơ bộ): \\dotfill \\\\\n')
        new_lines.append('Đơn vị: \\dotfill \\\\\n')
        new_lines.append('Ngày bảo vệ: \\dotfill \\\\\n')
        new_lines.append('Điểm tổng kết: \\dotfill \\\\\n')
        new_lines.append('Nơi lưu trữ luận án: \\dotfill \\vspace{0.5cm} \\\\\n')
        new_lines.append('\\hline\n')
        new_lines.append('\\end{longtable}\n')
        new_lines.append('\\end{center}\n')
        new_lines.append('\\clearpage\n')
        
        # Skip the rest of the current NHIỆM VỤ block until \clearpage
        while i < len(lines) and not lines[i].strip() == '\\clearpage':
            i += 1
        i += 1 # skip \clearpage itself
        continue
        
    # 3. Skip DANH SÁCH HỘI ĐỒNG BẢO VỆ ĐỒ ÁN
    if not skip and line.strip() == '% 5. DANH SÁCH HỘI ĐỒNG BẢO VỆ ĐỒ ÁN':
        # Remove the previous % === line
        if len(new_lines) > 0 and new_lines[-1].startswith('% ===='):
            new_lines.pop()
        while i < len(lines) and not lines[i].strip() == '\\clearpage':
            i += 1
        i += 1 # skip \clearpage
        continue
        
    # 4. Skip \begin{declaration} ... \end{declaration}
    if not skip and line.strip() == '\\begin{declaration}':
        while i < len(lines) and not lines[i].strip() == '\\end{declaration}':
            i += 1
        i += 1 # skip \end{declaration}
        continue

    if not skip:
        new_lines.append(line)
        
    i += 1

with open('thesis.tex', 'w', encoding='utf-8') as f:
    f.writelines(new_lines)

