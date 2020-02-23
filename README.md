# using_lidar_matrix
FROM3DTO2B

3B nokta bulutunu 2B görüntü ile eşleştirmek için, önce 3B nokta bulutunu 2B yoğunluklu bir görüntüye dönüştürmemiz gerekir.
Lidar verisinin 2D şeklinde gösterilmesi iki aşamada sağlanır. İlki   "the camera projection" metoduyla lidar verisini bir düzleme oturtmaktır.İkinci aşama ise interpolation kullnarak 2B noktalardan bir yoğunluk görüntüsü çıkartılıyor . Yoğunluk değeri de görüntüyü yoğunluğa göre renklendirmek için kullanılır.

LİDAR VERİSİNİ DÜZLEME YANSITMA

"the camera model"
Pİnhole Camera Projection Prensibi ,LiDAR 3D noktalarının bir listesini bir LiDAR 2D yoğunluk görüntüsüne dönüştürmek için kullanılır.
