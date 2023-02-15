#include<iostream>
#include"DaHengCamera.h"

/**
 * @brief 构造函数初始化相机
*/
DaHengCamera::DaHengCamera(){
    status = GXInitLib();
    if(status != GX_STATUS_SUCCESS){
        std::cout<<"Init ERROE\n";
    }
    else{
        std::cout<<"Init Success\n";
    }

}

/**
 * @brief 析构函数
 */
DaHengCamera::~DaHengCamera(){
    status = GXStreamOff(hDevice);
    status = GXCloseDevice(hDevice);
    status = GXCloseLib();

    if (status == GX_STATUS_SUCCESS){
        std::cout<<"Close Device";
    }
}

/**
 * @brief 打开相机
 * @param serial_number为要打开设备的序列号
 * @return 返回检测到的连接相机个数
 */
int DaHengCamera::StartDevice(int DeviceIndex){
    uint32_t nDeviceNum = 0;
    //枚 举 设 备 列 表
    status = GXUpdateDeviceList(&nDeviceNum, 1000);
    if(DeviceIndex > DeviceNum){
        std::cout<<"设备号错误，超过所枚举数量"<<std::endl;
        return -1;
    }
    //打 开 设 备
    status = GXOpenDeviceByIndex(DeviceIndex, &hDevice);
    if(status == GX_STATUS_SUCCESS){
        std::cout<<"设备打开成功!"<<std::endl;
        return nDeviceNum;
    }else{
        std::cout<<"设备打开失败!"<<std::endl;
        return -1;
    }
}

/**
 * @brief 序列号获取设备也可序号
 * @return 成功返回1跳出尝试
 */
bool DaHengCamera::StartDevice(){
    status = GXUpdateAllDeviceList(&DeviceNum, 1000);
    if(status != GX_STATUS_SUCCESS){
        return false;
    }
    streamOpenParam.accessMode = GX_ACCESS_EXCLUSIVE; //独占方式
    //stOpenParam.accessMode = GX_ACCESS_READONLY;//只读
    //stOpenParam.accessMode = GX_ACCESS_CONTROL;

//    streamOpenParam.openMode = GX_OPEN_SN; //序列号方式打开
//    streamOpenParam.pszContent = pszContent;

    streamOpenParam.openMode = GX_OPEN_INDEX; //枚举号方式打开
    streamOpenParam.pszContent = "1";
    status = GXOpenDevice(&streamOpenParam, &hDevice);
    if(status == GX_STATUS_SUCCESS){
        std::cout<<"设备打开成功!"<<std::endl;
        return true;
    }else{
        std::cout<<"设备打开失败!"<<std::endl;
        return false;
    }
}
/**
 * @brief 分辨率设置
 * @param ScaleReduction 缩小系数
 * @return 返回是否成功
 */
bool DaHengCamera::SetResolution(){

    status = GXSetEnum(hDevice,GX_ENUM_BINNING_HORIZONTAL_MODE,GX_BINNING_HORIZONTAL_MODE_SUM);
    status = GXSetEnum(hDevice,GX_ENUM_BINNING_VERTICAL_MODE,GX_BINNING_VERTICAL_MODE_SUM);
    status = GXSetInt(hDevice, GX_INT_BINNING_HORIZONTAL, ScaleReduction);
    status = GXSetInt(hDevice, GX_INT_BINNING_VERTICAL, ScaleReduction);
    status = GXSetInt(hDevice, GX_INT_DECIMATION_HORIZONTAL, ScaleReduction);
    status = GXSetInt(hDevice, GX_INT_DECIMATION_VERTICAL, ScaleReduction);
    if(status == GX_STATUS_SUCCESS){
        return true;
    }
    else{
        return false;
    }
}
/**
 * @brief 开启视频流
 *
 */
bool DaHengCamera::StreamOn(){
    //开 采
    status = GXStreamOn(hDevice);
    if(status == GX_STATUS_SUCCESS){
        std::cout<<"开始采集图像!\n";
        return true;
    }else{
        std::cout<<"采集开启失败!\n";
        return false;
    }
}
/**
 * @brief 获取帧大小
 *
 */
void DaHengCamera::getImageScale(int & width,int & height){
    width = pFrameBuffer->nWidth;
    height = pFrameBuffer->nHeight;
}


/**
 * @brief DaHengCamera::SetExposureTime 设置曝光值
 * @param ExposureTime  具体曝光值
 * @return bool 返回是否设置成功
 */
bool DaHengCamera::SetExposureTime(){
    //设 置  曝 光 值
    status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, exp_time);
    if(status == GX_STATUS_SUCCESS){
        std::cout<<"曝光值设置成功"<<std::endl;
        return true;
    }else{
        std::cout<<"曝光值设置失败"<<std::endl;
        return false;
    }
}

bool DaHengCamera::SetExposureTime(unsigned short time){
    //设 置  曝 光 值
    status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, time);
    if(status == GX_STATUS_SUCCESS){
        std::cout<<"曝光值设置成功"<<std::endl;
        return true;
    }else{
        std::cout<<"曝光值设置失败"<<std::endl;
        return false;
    }
}

/**
 * @brief DaHengCamera::SetGAIN 手动设置曝光增益
 * @param channel 选择曝光增益通道 0-B,1-G,2-R,3-All
 * @param ExpGain   具体增益值 范围0-16
 * @return
 */
bool DaHengCamera::SetGain(){

    switch(Exp_Channel){
        case Channel::BLUE:{
            status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_BLUE);
            break;
        }
        case Channel::RED:{
            status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_RED);
            break;
        }
        case Channel::GREEN:{
            status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_GREEN);
            break;
        }
        case Channel::ALL:{
            status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
            break;
        }
    }
    //设 置  曝 光 值
    status = GXSetFloat(hDevice, GX_FLOAT_GAIN, ExpGain);
    if(status == GX_STATUS_SUCCESS)
        return true;
    else
        return false;
}

bool DaHengCamera::SetGain(int value,int ExpGain){
    if(value == 0){
        //选 择 增 益 通 道 类 型
        status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_BLUE);
    }else if(value == 1){
        status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_GREEN);
    }else if(value == 2){
        status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_RED);
    }else{
        status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
    }
    //设 置  曝 光 值
    status = GXSetFloat(hDevice, GX_FLOAT_GAIN, ExpGain);
    if(status == GX_STATUS_SUCCESS)
        return true;
    else
        return false;
}


/**
 * @brief DaHengCamera::Set_BALANCE_AUTO 枚举变量为0是表示关闭，1为开启，具体请查询SDK手册,具有记忆功能
 * @return bool 返回是否设置成功
 */
bool DaHengCamera::Set_BALANCE_AUTO(int value){
    //设 置 连 续 自 动 白 平 衡
    status = GXSetEnum(hDevice,GX_ENUM_BALANCE_WHITE_AUTO,value);
    if(status == GX_STATUS_SUCCESS){
        std::cout<<"自动白平衡设置成功"<<std::endl;
        return true;
    }else{
        std::cout<<"自动白平衡设置失败"<<std::endl;
        return false;
    }
}

/**
 * @brief DaHengCamera::Set_BALANCE 手动白平衡,设置之前必须先关闭自动白平衡,具有记忆功能
 * @param value 选择平衡通道 0-B,1-G,2-R
 * @param value_number 平衡系数
 * @return
 */
bool DaHengCamera::Set_BALANCE(){


    switch(Balance_Channel){

        case Channel::BLUE:{
        status = GXSetEnum(hDevice,GX_ENUM_BALANCE_RATIO_SELECTOR,GX_BALANCE_RATIO_SELECTOR_BLUE);
            break;
        }
        case Channel::RED:{
        status = GXSetEnum(hDevice,GX_ENUM_BALANCE_RATIO_SELECTOR,GX_BALANCE_RATIO_SELECTOR_RED);
            break;
        }
        case Channel::GREEN:{
            status = GXSetEnum(hDevice,GX_ENUM_BALANCE_RATIO_SELECTOR,GX_BALANCE_RATIO_SELECTOR_GREEN);
            break;
        }
    }
    status = GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, Balence_value/10.0);
    if(status == GX_STATUS_SUCCESS)
        return true;
    else
        return false;
}


/**
 * @brief DaHengCamera::SetGamma 手动设置Gamma参数，调整其大小可看清装甲板数字
 * @param gamma 选择要用的gamma参数
 * @return 当前设置是否成功
 */
bool DaHengCamera::setGamma(double gamma)
{
    //GX_STATUS status = GX_STATUS_SUCCESS;
    //Enables Gamma.
    status = GXSetBool(hDevice, GX_BOOL_GAMMA_ENABLE, true);
    //Sets Gamma mode to user-defined mode.
    GX_GAMMA_MODE_ENTRY nValue;
    nValue = GX_GAMMA_SELECTOR_SRGB;
    status = GXSetEnum(hDevice, GX_ENUM_GAMMA_MODE, nValue);
    //Gets the Gamma parameter value.
    double dColorParam = gamma;
    status = GXSetFloat(hDevice, GX_FLOAT_GAMMA_PARAM, dColorParam);

    if(status){
        std::cout << "Gamma setted success !" << std::endl;
        return true;
    }else{
        std::cout << "Gamma setted failed !" << std::endl;
        return false;
    }

}

//bool DaHengCamera::Set_BALANCE(int balance_channel,int value){

////    Channel t_balance_channel = static_cast<Channel>(balance_channel);
//    Channel t_balance_channel = Channel(balance_channel);
//    switch(t_balance_channel){
//        case Channel::BLUE:{
//        status = GXSetEnum(hDevice,GX_ENUM_BALANCE_RATIO_SELECTOR,GX_BALANCE_RATIO_SELECTOR_BLUE);
//            break;
//        }
//        case Channel::RED:{
//        status = GXSetEnum(hDevice,GX_ENUM_BALANCE_RATIO_SELECTOR,GX_BALANCE_RATIO_SELECTOR_RED);
//            break;
//        }
//        case Channel::GREEN:{
//            status = GXSetEnum(hDevice,GX_ENUM_BALANCE_RATIO_SELECTOR,GX_BALANCE_RATIO_SELECTOR_GREEN);
//            break;
//        }default:{
//        }
//    }
//    status = GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, value/10.0);
//    if(status == GX_STATUS_SUCCESS)
//        return true;
//    else
//        return false;
//}

/**
 * @brief DaHengCamera::GetMat 读取图像
 * @param Src 引入方式传递
 * @return bool 返回是否成功
 */
/* bool DaHengCamera::GetMat(cv::Mat &Src){
    //调 用 GXDQBuf 取 一 帧 图 像
    status = GXDQBuf(hDevice, &pFrameBuffer, 1000);
      if(status == GX_STATUS_SUCCESS&&pFrameBuffer->nStatus == GX_FRAME_STATUS_SUCCESS){
           Src.create(pFrameBuffer->nHeight-1,pFrameBuffer->nWidth-1,CV_8UC3);
           uint8_t *t = (uint8_t *)pFrameBuffer->pImgBuf;
           for(int i = 0;i<Src.rows;i++){
               cv::Vec3b *p = Src.ptr<cv::Vec3b>(i);
               for(int j = 0;j<Src.cols;j++){
                   if(i%2 == 0){
                       if(j%2 == 0){
                           p[j][0] = t[j+1+(i+1)*pFrameBuffer->nWidth];
                           p[j][1] = (t[j+1+i*pFrameBuffer->nWidth] + t[j+(i+1)*pFrameBuffer->nWidth])/2;
                           p[j][2] = t[j+i*pFrameBuffer->nWidth];
                       }else{
                           p[j][0] = t[j+(i+1)*pFrameBuffer->nWidth];
                           p[j][1] = (t[j+1+(i+1)*pFrameBuffer->nWidth] + t[j+i*pFrameBuffer->nWidth])/2;
                           p[j][2] = t[j+1+i*pFrameBuffer->nWidth];
                       }
                   }else{
                       if(j%2 == 0){
                           p[j][0] = t[j+1+i*pFrameBuffer->nWidth];
                           p[j][1] = (t[j+1+(i+1)*pFrameBuffer->nWidth] + t[j+i*pFrameBuffer->nWidth])/2;
                           p[j][2] = t[j+(i+1)*pFrameBuffer->nWidth];
                       }else{
                           p[j][0] = t[j+i*pFrameBuffer->nWidth];
                           p[j][1] = (t[j+(i+1)*pFrameBuffer->nWidth] + t[j+1+i*pFrameBuffer->nWidth])/2;
                           p[j][2] = t[j+1+(i+1)*pFrameBuffer->nWidth];
                       }
                   }
               }
           }
           //调 用 GXQBuf 将 图 像 buf 放 回 库 中 继 续 采 图
           status = GXQBuf(hDevice, pFrameBuffer);
           return true;
      }else{
          std::cout<<"读取图片缓冲失败"<<std::endl;
          return false;
      }
} */

/**
 * @brief DaHengCamera::GetMat 读取图像
 * @param Src 引入方式传递
 * @return bool 返回是否成功
 */
bool DaHengCamera::GetMat(cv::Mat &Src)
{
    //调 用 GXDQBuf 取 一 帧 图 像
    status = GXDQBuf(hDevice, &pFrameBuffer, 1000);

    if(status == GX_STATUS_SUCCESS&&pFrameBuffer->nStatus == GX_FRAME_STATUS_SUCCESS){

        int64_t g_i64ColorFilter = GX_COLOR_FILTER_NONE;    ///< Color filter of device
        GXGetEnum(hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &g_i64ColorFilter);
        uint8_t *pRGB24Buf = new uint8_t[pFrameBuffer->nWidth * pFrameBuffer->nHeight * 3];
        uint8_t *pRaw8Image = new uint8_t[pFrameBuffer->nWidth * pFrameBuffer->nHeight];
        //DxRaw8toRGB24(pFrameBuffer,pRGB24Buf,pFrameBuffer->nWidth,pFrameBuffer->nHeight,RAW2RGB_NEIGHBOUR,BAYERRG,true);

        switch (pFrameBuffer->nPixelFormat){
            case GX_PIXEL_FORMAT_BAYER_GR8:
            case GX_PIXEL_FORMAT_BAYER_RG8:
            case GX_PIXEL_FORMAT_BAYER_GB8:
            case GX_PIXEL_FORMAT_BAYER_BG8:
        {
            // Convert to the RGB image
            DxRaw8toRGB24((unsigned char*)pFrameBuffer->pImgBuf, pRGB24Buf, pFrameBuffer->nWidth, pFrameBuffer->nHeight,RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(g_i64ColorFilter), false);
            break;
        }
            case GX_PIXEL_FORMAT_BAYER_GR10:
            case GX_PIXEL_FORMAT_BAYER_RG10:
            case GX_PIXEL_FORMAT_BAYER_GB10:
            case GX_PIXEL_FORMAT_BAYER_BG10:
            case GX_PIXEL_FORMAT_BAYER_GR12:
            case GX_PIXEL_FORMAT_BAYER_RG12:
            case GX_PIXEL_FORMAT_BAYER_GB12:
            case GX_PIXEL_FORMAT_BAYER_BG12:
        {
            // Convert to the Raw8 image
            DxRaw16toRaw8((unsigned char*)pFrameBuffer->pImgBuf, pRaw8Image, pFrameBuffer->nWidth, pFrameBuffer->nHeight, DX_BIT_2_9);
            // Convert to the RGB24 image
            DxRaw8toRGB24((unsigned char*)pRaw8Image, pRGB24Buf, pFrameBuffer->nWidth, pFrameBuffer->nHeight,RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(g_i64ColorFilter), false);
            break;
        }
        default:
        {
            printf("Error : PixelFormat of this camera is not supported\n");
        }
    }

        cv::Mat tMat(cv::Size(pFrameBuffer->nWidth,pFrameBuffer->nHeight),CV_8UC3,pRGB24Buf);
        cv::cvtColor(tMat,Src,cv::COLOR_RGB2BGR);

        //调 用 GXQBuf 将 图 像 buf 放 回 库 中 继 续 采 图
        status = GXQBuf(hDevice, pFrameBuffer);

        delete []pRGB24Buf;
        delete []pRaw8Image;
        return true;
    }
    else{
        std::cout<<"读取图片缓冲失败"<<std::endl;
        return false;
    }
}

