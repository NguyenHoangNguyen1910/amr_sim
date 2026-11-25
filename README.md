# Chạy mô phỏng Reeman 8020

Cần Docker và Docker Compose trên máy x86-64. Trên Windows, mở Docker Desktop ở chế độ Linux containers.

```bash
git clone https://github.com/NguyenHoangNguyen1910/amr_sim.git
cd amr_sim
docker compose -f docker-compose.reeman.yml up --wait
```

Mở [http://127.0.0.1:8020/](http://127.0.0.1:8020/).
