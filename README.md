# 🧠 Fram 3D Printer (AI + ROS2 + OctoPrint)

โครงการนี้เป็นระบบควบคุมเครื่องพิมพ์ 3 มิติ ที่ทำงานร่วมกับ **ROS2 Jazzy** และ **OctoPrint Deploy**  
รองรับการสั่งงานผ่าน API และ Node ROS2 สำหรับควบคุมการพิมพ์อัตโนมัติด้วย AI

---

## 📦 สิ่งที่ต้องมีก่อนใช้งาน

### 🔹 OctoPrint Deploy  
ใช้สำหรับติดตั้งและจัดการ OctoPrint Server  
📺 [ดูวิดีโอสอนการติดตั้ง](https://youtu.be/nDwW3eNxp2k?si=TrsdbVOf_T1Uyb2B)

### 🔹 ROS2 Jazzy  
เอกสารติดตั้งอย่างเป็นทางการ  
📘 [https://docs.ros.org/en/jazzy/Installation.html](https://docs.ros.org/en/jazzy/Installation.html)

---

# ⚙️ วิธีติดตั้ง

## 1. Clone โปรเจกต์
```
git clone https://github.com/Knnthiti/Fram_3D_printer.git
```

## 2. เข้าไปในโฟลเดอร์โปรเจกต์
```
cd Fram_3D_printer
```

## 3. โหลด environment ของ ROS2 Jazzy
```
source /opt/ros/jazzy/setup.bash
```

# 🚀 วิธีใช้งาน
## 1. สร้าง workspace และ build โปรเจกต์
```
colcon build
```
## 2. โหลด environment หลัง build เสร็จ
```
source install/setup.bash
```
## 3. รัน API Server ของระบบ
```
ros2 run ai_3dprint api_web

```
## 4.เปิด terminal ใหม่ แล้วรันคำสั่งนี้เพื่อเริ่มการตรวจจับการพิมพ์ 3D:
```
ros2 run ai_3dprint three_d_print
```
