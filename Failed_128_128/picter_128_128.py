import cv2
import os

# 📂 โฟลเดอร์ input และ output
input_folder = "input_images"
output_folder = "output_images"

# สร้างโฟลเดอร์ output ถ้ายังไม่มี
os.makedirs(output_folder, exist_ok=True)

# loop ไฟล์ทั้งหมดใน input folder
for filename in os.listdir(input_folder):
    # ตรวจว่าเป็นไฟล์ภาพ
    if filename.lower().endswith((".png", ".jpg", ".jpeg", ".bmp")):
        filepath = os.path.join(input_folder, filename)

        # โหลดภาพ
        img = cv2.imread(filepath)
        if img is None:
            print(f"❌ อ่านรูปไม่ได้: {filename}")
            continue

        # resize → 128x128
        resized = cv2.resize(img, (128, 128))

        # save ไปยัง output folder
        save_path = os.path.join(output_folder, filename)
        cv2.imwrite(save_path, resized)

        print(f"✅ resized {filename} → {save_path}")

print("🎉 เสร็จสิ้น! รูปทั้งหมดถูกปรับขนาดเป็น 128x128 แล้ว")
