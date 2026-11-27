# Chạy mô phỏng Reeman 8020

Nhánh này có bản thử nghiệm cảm biến trực tiếp ở cổng 8021. Xem [cách dựng và chạy bản thử nghiệm](README-lab.md).

Cần Docker và Docker Compose trên máy x86-64. Trên Windows, mở Docker Desktop ở chế độ Linux containers.

```bash
git clone https://github.com/NguyenHoangNguyen1910/amr_sim.git
cd amr_sim
docker compose -f docker-compose.reeman.yml up --wait
```

Mở [http://127.0.0.1:8020/](http://127.0.0.1:8020/).

## Mô phỏng chạy như thế nào?

1. **Clone từ GitHub.** Máy người dùng nhận giao diện, script mô phỏng và file `docker-compose.reeman.yml`.
2. **Chạy Docker Compose.** Compose đọc cấu hình và tải hai image từ Docker Hub nếu máy chưa có. `noetic-8020` chứa ROS Noetic và các node Reeman gốc. `gazebo-8020` chứa Gazebo.
3. **Chạy mô phỏng trên máy người dùng.** Gazebo giả lập robot và cảm biến. Script trong repo, ví dụ `hins_sim.py`, chuyển dữ liệu sang dạng driver Reeman đọc được. ROS xử lý dữ liệu và gửi lệnh chạy bánh xe về Gazebo.
4. **Xem và điều khiển qua web.** Dashboard trên cổng `8020` lấy dữ liệu từ hai container đang chạy trên máy người dùng.
