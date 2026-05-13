import re
import sys

def main():
    file_path = 'thesis/presentation/slide.tex'
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    appendix_start = content.find(r'\section*{Phụ lục}')
    if appendix_start == -1:
        print("Could not find \\section*{Phụ lục}")
        return

    main_content = content[:appendix_start + len(r'\section*{Phụ lục}') + 1]
    appendix_content = content[appendix_start + len(r'\section*{Phụ lục}') + 1:]

    # Extract all frames
    frame_pattern = re.compile(r'\\begin\{frame\}.*?\\end\{frame\}', re.DOTALL)
    frames = frame_pattern.findall(appendix_content)

    # Rest of the content after frames (like \end{document})
    end_doc_idx = appendix_content.find(r'\end{document}')
    end_doc_content = appendix_content[end_doc_idx:]

    # Map old titles to new titles
    mapping = {
        'Phụ lục A:': ('A', 'A1:'),
        'Phụ lục B:': ('B', 'A2:'),
        'Phụ lục F3:': ('F3', 'A3:'),
        
        'Phụ lục F4:': ('F4', 'B1:'),
        'Phụ lục E1:': ('E1', 'B2:'),
        'Phụ lục E2:': ('E2', 'B3:'),
        'Phụ lục F7:': ('F7', 'B4:'),
        
        'Phụ lục F2:': ('F2', 'C1:'),
        'Phụ lục F6:': ('F6', 'C2:'),
        'Phụ lục F8:': ('F8', 'C3:'),
        
        'Phụ lục F1:': ('F1', 'D1:'),
        'Phụ lục C:': ('C', 'D2:'),
        'Phụ lục F10:': ('F10', 'D3:'),
        'Phụ lục F11:': ('F11', 'D4:'),
        'Phụ lục D:': ('D', 'D5:'),
        'Phụ lục F12:': ('F12', 'D6:'),
        'Phụ lục F5:': ('F5', 'D7:'),
        'Phụ lục F13:': ('F13', 'D8:'),
        
        'Phụ lục E3:': ('E3', 'E1:'),
        'Phụ lục F9:': ('F9', 'E2:'),
    }

    frame_dict = {}
    for frame in frames:
        # Find which mapping it matches
        matched = False
        for old_key, val in mapping.items():
            if old_key in frame:
                # Replace the old prefix with the new prefix
                new_frame = frame.replace(old_key, 'Phụ lục ' + val[1])
                frame_dict[val[1]] = new_frame
                matched = True
                break
        if not matched:
            print(f"Warning: frame not matched! {frame[:50]}")
            
    # Fix the lengths in A1
    a1_frame = frame_dict['A1:']
    a1_frame = a1_frame.replace(
        r'L_1=45\,\text{mm}, L_2=107\,\text{mm}, L_3=116\,\text{mm}',
        r'L_1=26\,\text{mm}, L_2=106\,\text{mm}, L_3=125\,\text{mm}'
    )
    frame_dict['A1:'] = a1_frame

    # Construct the new appendix content
    ordered_keys = [
        'A1:', 'A2:', 'A3:',
        'B1:', 'B2:', 'B3:', 'B4:',
        'C1:', 'C2:', 'C3:',
        'D1:', 'D2:', 'D3:', 'D4:', 'D5:', 'D6:', 'D7:', 'D8:',
        'E1:', 'E2:'
    ]
    
    new_appendix_str = "\n\n"
    for k in ordered_keys:
        if k in frame_dict:
            new_appendix_str += frame_dict[k] + "\n\n"
        else:
            print(f"Missing key: {k}")

    # Combine everything
    final_content = main_content + new_appendix_str + end_doc_content

    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(final_content)
        
    print("Successfully reordered and updated frames.")

if __name__ == '__main__':
    main()
