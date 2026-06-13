# 3 Serbestlik Dereceli (3-DOF) Robotik Manipülatör - ROS2 & MoveIt2

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg) ![MoveIt2](https://img.shields.io/badge/MoveIt2-Y%C3%B6r%C3%BCnge%20Planlama-orange.svg) ![Hardware](https://img.shields.io/badge/Donan%C4%B1m-ESP32-brightgreen.svg)

*Read this in [English](README.md).*

## Genel Bakış
Bu proje, sıfırdan tasarlanıp üretilmiş **3 Serbestlik Dereceli (3-DOF) Seri Robot Manipülatörü**'nün tam donanım ve yazılım mimarisini içermektedir. Sistem, ileri kinematik hesaplamaları, gerçek zamanlı donanım kontrolünü ve **ROS2** ile **MoveIt2** kullanılarak yapılan yüksek seviyeli yörünge planlamasını bir araya getirmektedir.

Proje; 3 boyutlu mekanik tasarım (Autodesk Inventor), gömülü sistemler (ESP32/Arduino) ve Robotik İşletim Sistemleri (ROS2) alanlarındaki çok disiplinli mühendislik yetkinliklerini sergilemektedir.

## Anahtar Özellikler

*   **Özel Mekanik Tasarım:** Autodesk Inventor kullanılarak modellenmiş eklemli robot kolu tasarımı. Parçalar tork ve yapısal bütünlük (rigidity) hesaba katılarak 3D yazıcı ile üretilmiştir.
*   **ROS2 Mimarisi:** Modüler ROS2 çalışma alanı (workspace) şu paketleri içerir:
    *   `mechanism_308_description`: RViz'de 3D dijital ikiz (Digital Twin) için URDF modelleri.
    *   `mechanism_308_moveit_config`: Çarpışma algılayıcı ters kinematik (IK) ve hareket planlaması için MoveIt2 konfigürasyonu.
    *   `mechanism_308_hardware`: Dijital ikiz ile fiziksel ESP32 denetleyicileri arasında köprü kuran özel `ros2_control` donanım arayüzü.
*   **Denavit-Hartenberg (DH) & Kinematik:** İleri ve ters kinematik, DH parametreleri üzerinden parametrik olarak çözülmüştür. Yörünge planlamasında "tekillik" (singularity) sorunlarından kaçınmak için Jacobian matris analizleri yapılmıştır.
*   **Serbest Sürüş (Free-Drive) & Öğretme Modu:** Motor torklarını keserek robotun insan eliyle fiziksel olarak yönlendirilmesine olanak tanıyan özel Python düğümü (`free_drive.py`). Bu esnada eklem açıları gerçek zamanlı olarak RViz'e aktarılır, böylece robota fiziksel yörünge öğretilebilir ve dinamik sıfır (Home) noktası atanabilir.
*   **Gerçek Zamanlı HIL (Hardware-in-the-Loop):** Windows (WSL) ve ESP32 mikrodenetleyicileri arasındaki seri haberleşme yönlendirmesi sayesinde, MoveIt2'nin ürettiği eklem uzayı yörüngeleri düşük gecikmeyle (low-latency) donanıma aktarılır.

## Klasör Yapısı

*   `CAD-Models/`: Evrensel uyumluluk için dışa aktarılmış, ROS2 adlandırmasıyla senkronize `mechanism_308.step` 3D modeli.
*   `ROS2-Workspace/`: ROS2 paketlerini ve Python yürütme düğümlerini içeren ana `colcon` çalışma alanı.
*   `photos/`: Fiziksel robotun donanım görselleri.
*   `Motor_Test/` & `Motor_PID_Test/`: PID hız/konum kontrolü ve enkoder okumaları için ESP32'ye gömülü C/C++ yazılımları.
