# KTH5701
Kth5701磁编码传感器android驱动以及应用

## 设备树的配置
&i2c1{

	#address-cells = <1>;
 
	#size-cells = <0>;
 
	clock-frequency = <400000>;
 
	status = "okay";
 
	kth5701@68 {
 
		compatible = "magencoder,kth5701";
  
		reg = <0x68>;
  
		enable-gpio = <&pio 171 0>;
  
		hall-eint = <&pio 1 0>;
  
	};
 
};


## 节点的selinux权限修改
详见：kth5701_selinux.patch

## 运行效果图
![image text](https://github.com/hgddingjun/KTH5701/blob/main/kth5701_drv_app/pic/snapshot.png "magnetic encoder")
