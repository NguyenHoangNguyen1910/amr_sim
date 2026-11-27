# Bản thử nghiệm Reeman 8021

Nhánh `codex/direct-gazebo-sensors` của `amr_sim`, phát triển từ commit `63f3b35`. Dùng Docker Desktop với Linux containers. Image Noetic dùng nguyên tag Docker Hub. Image Gazebo dẫn xuất chỉ thêm plugin C++.

## Chạy

Mở PowerShell và clone nhánh thử nghiệm.

```powershell
git clone --branch codex/direct-gazebo-sensors https://github.com/NguyenHoangNguyen1910/amr_sim.git amr_sim_direct
cd amr_sim_direct
```

Nếu đã có nhánh này trên máy, mở PowerShell tại thư mục nhánh.

```powershell
docker build -f src/reeman/docker/Dockerfile.direct -t reeman-direct-gazebo:local .
docker compose -f docker-compose.lab.yml up --wait
```

Mở [giao diện mô phỏng](http://127.0.0.1:8021/). Cổng WebSocket là `9013`.

```powershell
docker compose -f docker-compose.lab.yml down
```

## So sánh các nhánh

Mặc định dùng cả ba nhánh trực tiếp. Đặt `$env:REEMAN_DIRECT_DEVICES='depth'` để chỉ dùng camera trực tiếp. Có thể dùng danh sách phân tách bằng dấu phẩy gồm `depth,lidar,imu`. Đặt `$env:REEMAN_DIRECT_DEVICES='bridge'` để dùng lại bridge cho cả ba cảm biến. Sau đó chạy `docker compose -f docker-compose.lab.yml stop` và `docker compose -f docker-compose.lab.yml up --force-recreate --wait` để cả hai container đọc launch của phiên mới.

- Camera gửi frame trực tiếp tới thiết bị OpenNI ảo hiện có và vẫn đi qua driver Astra.
- LiDAR mở TCP trong mạng nội bộ Compose cho driver HINS-HE. Relay phía Noetic ghép lại thời gian đo từ Gazebo vào scan do driver giải mã.
- IMU gửi mẫu đến bộ giả lập serial trong Noetic và giữ driver WIT901.
- Bridge còn lại giữ lệnh đế, dừng cưỡng bức, `/mobile/odom`, `/clock`, joint states và các tín hiệu khác của đế.

## Kiểm thử

Chạy sau khi mở phòng mặc định, khi robot rảnh và chưa quay về phía dưới bản đồ. Bài kiểm tra có di chuyển robot trong mô phỏng và gỡ vật cản nó tạo sau khi đo.

```powershell
docker compose -f docker-compose.lab.yml exec -T reeman-noetic bash -lc 'source /opt/ros/noetic/setup.bash && source /home/reeman/catkin_ws/install_isolated/setup.bash && source /home/reeman/reeman_ws/install/setup.bash && python3 -u /sim/smoke.py --plan'
docker compose -f docker-compose.lab.yml exec -T reeman-noetic bash -lc 'source /opt/ros/noetic/setup.bash && source /home/reeman/reeman_ws/install/setup.bash && python3 -u /sim/verify_direct.py --a2'
```
