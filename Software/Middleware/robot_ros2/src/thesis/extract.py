import PyPDF2
with open("DD_Huong dan viet TM_DATN_2026May05.pdf", "rb") as f:
    reader = PyPDF2.PdfReader(f)
    text = ""
    for page in reader.pages:
        text += page.extract_text() + "\n"

print("--- ALL TEXT ---")
print(text)
