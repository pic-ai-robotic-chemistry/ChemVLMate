import sys
from ctypes import *
import os
import numpy as np
import time
import cv2

sys.path.append("./MvImport") #打开MVS中的MvImport文件,对于不同系统打开的文件路径跟随实际文件路径变化即可
from MvCameraControl_class import * #调用了MvCameraControl_class.py文件


class HKCamera():
    def __init__(self, CameraIdx=0, log_path=None):
        # enumerate all the camera devices
        deviceList = self.enum_devices()
        self.stop_capturing = False

        # generate a camera instance
        self.camera = self.open_camera(deviceList, CameraIdx, log_path)
        self.start_camera()

    def __del__(self):
        # 结束循环取流
        self.stop_capturing = True
        # 关闭cv2窗口
        cv2.destroyAllWindows()
        if self.camera is None:
            return

        # 停止取流
        ret = self.camera.MV_CC_StopGrabbing()
        if ret != 0:
            raise Exception("stop grabbing fail! ret[0x%x]" % ret)

        # 关闭设备
        ret = self.camera.MV_CC_CloseDevice()
        if ret != 0:
            raise Exception("close deivce fail! ret[0x%x]" % ret)

        # 销毁句柄
        ret = self.camera.MV_CC_DestroyHandle()
        if ret != 0:
            raise Exception("destroy handle fail! ret[0x%x]" % ret)

    @staticmethod
    def enum_devices(device=0, device_way=False):
        """
        device = 0  枚举网口、USB口、未知设备、cameralink 设备
        device = 1 枚举GenTL设备
        """
        if device_way == False:
            if device == 0:
                cameraType = MV_GIGE_DEVICE | MV_USB_DEVICE | MV_UNKNOW_DEVICE | MV_1394_DEVICE | MV_CAMERALINK_DEVICE
                deviceList = MV_CC_DEVICE_INFO_LIST()
                # 枚举设备
                ret = MvCamera.MV_CC_EnumDevices(cameraType, deviceList)
                if ret != 0:
                    raise Exception("enum devices fail! ret[0x%x]" % ret)
                return deviceList
            else:
                pass
        elif device_way == True:
            pass

    def open_camera(self, deviceList, CameraIdx, log_path):
        # generate a camera instance
        camera = MvCamera()

        # 选择设备并创建句柄
        stDeviceList = cast(deviceList.pDeviceInfo[CameraIdx], POINTER(MV_CC_DEVICE_INFO)).contents
        if log_path is not None:
            ret = self.camera.MV_CC_SetSDKLogPath(log_path)
            if ret != 0:
                raise Exception("set Log path  fail! ret[0x%x]" % ret)

            # 创建句柄,生成日志
            ret = camera.MV_CC_CreateHandle(stDeviceList)
            if ret != 0:
                raise Exception("create handle fail! ret[0x%x]" % ret)
        else:
            # 创建句柄,不生成日志
            ret = camera.MV_CC_CreateHandleWithoutLog(stDeviceList)
            if ret != 0:
                raise Exception("create handle fail! ret[0x%x]" % ret)
        if ret != 0:
            raise Exception("Set BalanceWhiteAuto faill! ter[0x%x]" % ret)
        # 打开相机
        ret = camera.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if ret != 0:
            raise Exception("open device fail! ret[0x%x]" % ret)
        # # 探测网络最佳包大小（只对GigE相机有效）
        # if stDeviceList.nTLayerType == MV_GIGE_DEVICE:
        #     nPacketSize = camera.MV_CC_GetOptimalPacketSize()
        #     if int(nPacketSize)>0:
        #         ret = camera.MV_CC_SetIntValue("GevSCPSPacketSize",nPacketSize)
        #         if ret != 0:
        #             raise Exception("Warning: Set Packet Size faill! ret[0x%x]" % ret)
        #     else:
        #         raise Exception("Warning: Get Packet Size faill! ret[0x%x]" % nPacketSize)
        # 设置触发模式为off
        ret = camera.MV_CC_SetEnumValue("TriggerMode",MV_TRIGGER_MODE_OFF)
        if ret != 0:
            raise Exception("set TriggerMode fail! ret[0x%x]" % ret)
        # 关闭自动曝光时间
        ret = camera.MV_CC_SetEnumValue("ExposureAuto",MV_EXPOSURE_AUTO_MODE_OFF)
        if ret != 0:
            raise Exception("set ExposureAuto fail! ret[0x%x]" % ret)
        # 自动增益  连续模式
        ret = camera.MV_CC_SetEnumValue("GainAuto",MV_GAIN_MODE_CONTINUOUS)
        if ret != 0:
            raise Exception("set GainAuto fail! ret[0x%x]" % ret)
        # 增益值 范围 0 - 23.98dB
        # ret = camera.MV_CC_SetFloatValue("Gain",3)
        # if ret != 0:
        #     raise Exception("set Gain fail! ret[0x%x]" % ret)
        # 设置采集帧率 范围：0.1 - 100000
        ret = camera.MV_CC_SetFloatValue("AcquisitionFrameRate", float(50.00))
        if ret != 0:
            raise Exception("Set AcquisitionFrameRate fail! ter[0x%x]" % ret)
        # 设置曝光时间 范围 15 - 9999448
        ret = camera.MV_CC_SetFloatValue("ExposureTime", float(800.00))
        if ret != 0:
            raise Exception("Set ExposureTime fail! ter[0x%x]" % ret)
        # 设置自动白平衡
        ret = camera.MV_CC_SetEnumValue("BalanceWhiteAuto", 1)
        if ret !=0:
            raise Exception("Set BalanceWhiteAuto fail! ret[0x%x]" % ret)
        # 设置亮度 范围 0 - 255
        ret = camera.MV_CC_SetIntValue("Brightness", 80)
        if ret != 0:
            raise Exception("Set Brightness fail! ret[0x%x]" % ret)
        # 设置像素格式
        ret = camera.MV_CC_SetEnumValue("PixelFormat", 0x02180014)
        if ret != 0:
            raise Exception("Set PixelFormat fail! ret[0x%x]" % ret)

        return camera

    def start_camera(self):
        stParam = MVCC_INTVALUE()
        memset(byref(stParam), 0, sizeof(MVCC_INTVALUE))

        ret = self.camera.MV_CC_GetIntValue("PayloadSize", stParam)
        if ret != 0:
            raise Exception("get payload size fail! ret[0x%x]" % ret)

        self.nDataSize = stParam.nCurValue
        self.pData = (c_ubyte * self.nDataSize)()
        self.stFrameInfo = MV_FRAME_OUT_INFO_EX()
        memset(byref(self.stFrameInfo), 0, sizeof(self.stFrameInfo))

        self.camera.MV_CC_StartGrabbing()

    def get_Value(self, param_type, node_name):
        """
        :param cam:            相机实例
        :param_type:           获取节点值得类型
        :param node_name:      节点名 可选 int 、float 、enum 、bool 、string 型节点
        :return:               节点值
        """
        if param_type == "int_value":
            stParam = MVCC_INTVALUE_EX()
            memset(byref(stParam), 0, sizeof(MVCC_INTVALUE_EX))
            ret = self.camera.MV_CC_GetIntValueEx(node_name, stParam)
            if ret != 0:
                raise Exception("获取 int 型数据 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            return stParam.nCurValue

        elif param_type == "float_value":
            stFloatValue = MVCC_FLOATVALUE()
            memset(byref(stFloatValue), 0, sizeof(MVCC_FLOATVALUE))
            ret = self.camera.MV_CC_GetFloatValue(node_name, stFloatValue)
            if ret != 0:
                raise Exception("获取 float 型数据 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            return stFloatValue.fCurValue

        elif param_type == "enum_value":
            stEnumValue = MVCC_ENUMVALUE()
            memset(byref(stEnumValue), 0, sizeof(MVCC_ENUMVALUE))
            ret = self.camera.MV_CC_GetEnumValue(node_name, stEnumValue)
            if ret != 0:
                raise Exception("获取 enum 型数据 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            return stEnumValue.nCurValue

        elif param_type == "bool_value":
            stBool = c_bool(False)
            ret = self.camera.MV_CC_GetBoolValue(node_name, stBool)
            if ret != 0:
                raise Exception("获取 bool 型数据 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            return stBool.value

        elif param_type == "string_value":
            stStringValue = MVCC_STRINGVALUE()
            memset(byref(stStringValue), 0, sizeof(MVCC_STRINGVALUE))
            ret = self.camera.MV_CC_GetStringValue(node_name, stStringValue)
            if ret != 0:
                raise Exception("获取 string 型数据 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            return stStringValue.chCurValue

        else:
            return None

    def set_Value(self, param_type, node_name, node_value):
        """
        :param cam:               相机实例
        :param param_type:        需要设置的节点值得类型
            int:
            float:
            enum:     参考于客户端中该选项的 Enum Entry Value 值即可
            bool:     对应 0 为关，1 为开
            string:   输入值为数字或者英文字符，不能为汉字
        :param node_name:         需要设置的节点名
        :param node_value:        设置给节点的值
        :return:
        """
        if param_type == "int_value":
            ret = self.camera.MV_CC_SetIntValueEx(node_name, int(node_value))
            if ret != 0:
                raise Exception("设置 int 型数据节点 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))

        elif param_type == "float_value":
            ret = self.camera.MV_CC_SetFloatValue(node_name, float(node_value))
            if ret != 0:
                raise Exception("设置 float 型数据节点 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))

        elif param_type == "enum_value":
            ret = self.camera.MV_CC_SetEnumValue(node_name, node_value)
            if ret != 0:
                raise Exception("设置 enum 型数据节点 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))

        elif param_type == "bool_value":
            ret = self.camera.MV_CC_SetBoolValue(node_name, node_value)
            if ret != 0:
                raise Exception("设置 bool 型数据节点 %s 失败 ！ 报错码 ret[0x%x]" % (node_name, ret))

        elif param_type == "string_value":
            ret = self.camera.MV_CC_SetStringValue(node_name, str(node_value))
            if ret != 0:
                raise Exception("设置 string 型数据节点 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))

    def set_exposure_time(self, exp_time):
        self.set_Value(param_type="float_value", node_name="ExposureTime", node_value=float(exp_time))

    # 获取曝光时间值
    def get_exposure_time(self):
        return self.get_Value(param_type="float_value", node_name="ExposureTime")

    # 获取自动白平衡 1开启 0关闭
    def get_balance_white_auto(self):
        return self.get_Value(param_type="enum_value", node_name="BalanceWhiteAuto")

    def get_image(self, width=None):
        """
        :param cam:     相机实例
        :active_way:主动取流方式的不同方法 分别是（getImagebuffer）（getoneframetimeout）
        :return:
        """
        ret = self.camera.MV_CC_GetOneFrameTimeout(self.pData, self.nDataSize, self.stFrameInfo, 1000)
        if ret == 0:
            # image = np.asarray(self.pData).reshape((self.stFrameInfo.nHeight, self.stFrameInfo.nWidth, 3))
            image = np.array(self.pData)
            image = image.reshape((self.stFrameInfo.nHeight,self.stFrameInfo.nWidth,3))
            # image = np.asarray(self.pData).reshape((self.stFrameInfo.nHeight, self.stFrameInfo.nWidth,2))
            if width is not None:
                image = cv2.resize(image, (width, int(self.stFrameInfo.nHeight * width / self.stFrameInfo.nWidth)))
                pass
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            return image
        else:
            return None

    def show_runtime_info(self, image):
        exp_time = self.get_exposure_time()
        cv2.putText(image, ("exposure time = %1.1fms" % (exp_time * 0.001)), (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255, 1)

    def saveImage(self,filename):
        image = self.get_image()
        cv2.imwrite('./images/'+filename+'.jpg',image)

    def start(self):
        # while not self.stop_capturing:
        #  image = camera.get_image(width=800)
        #  if image is not None:
        #      camera.show_runtime_info(image)
        #      cv2.imshow("", image)
        try:

            while True:
                image = self.get_image(width=800)
                if image is not None:
                    self.show_runtime_info(image)
                    cv2.imshow("", image)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == ord('Q'):
                    cv2.destroyAllWindows()
                    break
        except Exception as e:
            print(e)

if __name__ == '__main__':

    camera = HKCamera()
    # camera.saveImage()
    print(camera.get_exposure_time())
    # camera.set_exposure_time(1234)
    # 42715.0
    print(camera.get_balance_white_auto())
    # camera.set_exposure_time(5000.0)
    camera.start()
    # try:
    #
    #     while True:
    #      image = camera.get_image(width=800)
    #      if image is not None:
    #          camera.show_runtime_info(image)
    #          cv2.imshow("", image)
    #      key = cv2.waitKey(1) & 0xFF
    #      if key == ord('q') or key == ord('Q'):
    #          cv2.destroyAllWindows()
    #          break
    # except Exception as e:
    #     print(e)
