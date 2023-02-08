#include "../../include/autoaim/inference.h"


bool Inference::detect(cv::Mat &src, std::vector<ArmorObject>& objects)
{
    if(src.empty())
    {
        std::cout << "the entry image is empty!!!" << std::endl;
        return false;
    }

    Eigen::Matrix<float,3,3> transform_matrix;
    cv::Mat pr_img = scaledResize(src, transform_matrix);

    uint8_t *input_data = (uint8_t *)pr_img.data;
    input_tensor = ov::Tensor(executable_network.input().get_element_type(), executable_network.input().get_shape(), input_data);
    infer_request.set_input_tensor(input_tensor);

    infer_request.infer();

    std::vector<float *> output_blobs;
    for (int idx = 0; idx < outputNode_size; idx++)
    {
        output_tensor = infer_request.get_output_tensor(idx);
        float *detections = output_tensor.data<float>();
        output_blobs.push_back(detections);
    }

    int img_w = src.cols;
    int img_h = src.rows;

    decodeOutputs(output_blobs, objects, transform_matrix, img_w, img_h);
    for (auto object = objects.begin(); object != objects.end(); ++object)
    {
        //对候选框预测角点进行平均,降低误差
        if ((*object).pts.size() >= 8)
        {
            auto N = (*object).pts.size();
            cv::Point2f pts_final[4];

            for (int i = 0; i < N; i++)
            {
                pts_final[i % 4]+=(*object).pts[i];
            }

            for (int i = 0; i < 4; i++)
            {
                pts_final[i].x = pts_final[i].x / (N / 4);
                pts_final[i].y = pts_final[i].y / (N / 4);
            }

            (*object).apex[0] = pts_final[0];
            (*object).apex[1] = pts_final[1];
            (*object).apex[2] = pts_final[2];
            (*object).apex[3] = pts_final[3];
        }
        (*object).area = (int)(calcTetragonArea((*object).apex));
    }
    if (objects.size() != 0)
        return true;
    else return false;


}

inline cv::Mat Inference::scaledResize(cv::Mat& img, Eigen::Matrix<float,3,3>& transform_matrix)
{
    cv::Mat RGB_img;
    cvtColor(img, RGB_img, cv::COLOR_BGR2RGB);
    cv::Mat re(INPUT_H, INPUT_W, CV_8UC3);
    cv::resize(RGB_img, re, re.size(), 0, 0, cv::INTER_LINEAR);
    return re;
}

void Inference::generate_grids_and_stride(const int target_w, const int target_h, std::vector<int>& strides, std::vector<GridAndStride>& grid_strides)
{
    for (auto stride : strides)
    {
        int num_grid_w = target_w / stride;
        int num_grid_h = target_h / stride;

        for (int g1 = 0; g1 < num_grid_h; g1++)
        {
            for (int g0 = 0; g0 < num_grid_w; g0++)
            {
                grid_strides.push_back((GridAndStride){g0, g1, stride});
            }
        }
    }
}

void Inference::generateYoloxProposals(std::vector<GridAndStride> grid_strides, const float* feat_ptr,
                                       Eigen::Matrix<float,3,3> &transform_matrix,float prob_threshold,
                                       std::vector<ArmorObject>& objects)
{
    const int num_anchors = grid_strides.size();
    //std::cout << "num_anchors:      " << num_anchors << std::endl;
    //Travel all the anchors
    for (int anchor_idx = 0; anchor_idx < num_anchors; anchor_idx++)
    {
        const int grid0 = grid_strides[anchor_idx].grid0;
        const int grid1 = grid_strides[anchor_idx].grid1;
        const int stride = grid_strides[anchor_idx].stride;

        const int basic_pos = anchor_idx * (9 + NUM_COLORS + NUM_CLASSES);

        // yolox/models/yolo_head.py decode logic
        //  outputs[..., :2] = (outputs[..., :2] + grids) * strides
        //  outputs[..., 2:4] = torch.exp(outputs[..., 2:4]) * strides
        float x_1 = (feat_ptr[basic_pos + 0] + grid0) * stride;
        float y_1 = (feat_ptr[basic_pos + 1] + grid1) * stride;
        float x_2 = (feat_ptr[basic_pos + 2] + grid0) * stride;
        float y_2 = (feat_ptr[basic_pos + 3] + grid1) * stride;
        float x_3 = (feat_ptr[basic_pos + 4] + grid0) * stride;
        float y_3 = (feat_ptr[basic_pos + 5] + grid1) * stride;
        float x_4 = (feat_ptr[basic_pos + 6] + grid0) * stride;
        float y_4 = (feat_ptr[basic_pos + 7] + grid1) * stride;

        int box_color = argmax(feat_ptr + basic_pos + 9, NUM_COLORS);
        int box_class = argmax(feat_ptr + basic_pos + 9 + NUM_COLORS, NUM_CLASSES);

        float box_objectness = (feat_ptr[basic_pos + 8]);

        float color_conf = (feat_ptr[basic_pos + 9 + box_color]);
        float cls_conf = (feat_ptr[basic_pos + 9 + NUM_COLORS + box_class]);

        // float box_prob = (box_objectness + cls_conf + color_conf) / 3.0;
        float box_prob = box_objectness;

        if (box_prob >= prob_threshold)
        {
            ArmorObject obj;

            Eigen::Matrix<float,3,4> apex_norm;
            Eigen::Matrix<float,3,4> apex_dst;

            apex_norm << x_1,x_2,x_3,x_4,
                    y_1,y_2,y_3,y_4,
                    1,1,1,1;

            apex_dst = transform_matrix * apex_norm;

            for (int i = 0; i < 4; i++)
            {
                obj.apex[i] = cv::Point2f(apex_dst(0,i),apex_dst(1,i));
                obj.pts.push_back(obj.apex[i]);
            }

            std::vector<cv::Point2f> tmp(obj.apex,obj.apex + 4);
            obj.rect = cv::boundingRect(tmp);

            obj.cls = box_class;
            obj.color = box_color;
            obj.prob = box_prob;

            objects.push_back(obj);
        }

    } // point anchor loop
}


void Inference::qsort_descent_inplace(std::vector<ArmorObject>& faceobjects, int left, int right)
{
    int i = left;
    int j = right;
    float p = faceobjects[(left + right) / 2].prob;

    while (i <= j)
    {
        while (faceobjects[i].prob > p)
            i++;

        while (faceobjects[j].prob < p)
            j--;

        if (i <= j)
        {
            // swap
            std::swap(faceobjects[i], faceobjects[j]);

            i++;
            j--;
        }
    }

#pragma omp parallel sections
    {
#pragma omp section
        {
            if (left < j) qsort_descent_inplace(faceobjects, left, j);
        }
#pragma omp section
        {
            if (i < right) qsort_descent_inplace(faceobjects, i, right);
        }
    }
}


void Inference::qsort_descent_inplace(std::vector<ArmorObject>& objects)
{
    if (objects.empty())
        return;

    qsort_descent_inplace(objects, 0, objects.size() - 1);
}

void Inference::nms_sorted_bboxes(std::vector<ArmorObject>& faceobjects, std::vector<int>& picked,
                                  float nms_threshold)
{
    picked.clear();

    const int n = faceobjects.size();

    std::vector<float> areas(n);
    for (int i = 0; i < n; i++)
    {
        areas[i] = faceobjects[i].rect.area();
    }

    for (int i = 0; i < n; i++)
    {
        ArmorObject& a = faceobjects[i];

        int keep = 1;
        for (int j = 0; j < (int)picked.size(); j++)
        {
            ArmorObject& b = faceobjects[picked[j]];

            // intersection over union
            float inter_area = intersection_area(a, b);
            float union_area = areas[i] + areas[picked[j]] - inter_area;
            float iou = inter_area / union_area;
            if (iou > nms_threshold || isnan(iou))
            {
                keep = 0;
                //Stored for Merge
                if (iou > MERGE_MIN_IOU && abs(a.prob - b.prob) < MERGE_CONF_ERROR
                        && a.cls == b.cls && a.color == b.color)
                {
                    for (int i = 0; i < 4; i++)
                    {
                        b.pts.push_back(a.apex[i]);
                    }
                }
                // cout<<b.pts_x.size()<<endl;
            }
        }

        if (keep)
            picked.push_back(i);
    }
}

void Inference::decodeOutputs(std::vector<float *>& output_blobs, std::vector<ArmorObject>& objects,
                              Eigen::Matrix<float,3,3> &transform_matrix, const int img_w, const int img_h)
{
    float* prob = static_cast<float*>(output_blobs[0]);
    std::vector<ArmorObject> proposals;
    std::vector<int> strides = {8, 16, 32};
    std::vector<GridAndStride> grid_strides;

    //generate_grids_and_stride(INPUT_W, INPUT_H, strides, grid_strides);
    //std::cout << grid_strides.size() << std::endl;
    float ratioh = img_h / INPUT_H, ratiow = img_w / INPUT_W;

    /// xmin,ymin,xamx,ymax,box_score,x1,y1, ... ,x5,y5,class_sorce...
    int n = 0,i = 0, q = 0,j = 0, nout = 51, row_ind = 0, k = 0; // 51 = 36 + 15

    for (n = 0; n < 3; n++)   ///特征图尺度
    {
        int num_grid_x = (int)(INPUT_W / strides[n]);
        int num_grid_y = (int)(INPUT_H / strides[n]);
        //std::cout << num_grid_x << std::endl;
        for(q = 0; q <3 ; q++)
        {
            for (i = 0; i < num_grid_y; i++)
            {
                for (j = 0; j < num_grid_x; j++)
                {
                    ArmorObject obj;
                    float* pdata = prob + row_ind * nout;
                    row_ind++;

                    float box_score = (pdata[4]);
                    if(box_score <= BBOX_CONF_THRESH)
                    {
                        continue;
                    }

                    int id = argmax(pdata + 15, NUM_CLASSES);
                    float armor_score = (pdata[15 + id]);

//                    if(armor_score < 0.5)
//                        continue;

                    float cx = pdata[0]*ratiow; ///cx
                    float cy = pdata[1]*ratioh; ///cy
                    float w = pdata[2];         ///w
                    float h = pdata[3];         ///h

                    int left = (cx - 0.5*w);
                    int top = (cy - 0.5*h);


                    //std::cout << box_score << std::endl;
                    std::vector<float> landmark(10);
                    for (k = 5; k < 13; k+=2)
                    {
                        const int ind = k - 5;
                        landmark[ind] = (float)(pdata[k])*ratiow;
                        landmark[ind + 1] = (float)(pdata[k + 1] )*ratioh;
                    }

                    Eigen::Matrix<float,3,4> apex_norm;
                    Eigen::Matrix<float,3,4> apex_dst;

                    apex_norm << landmark[1],landmark[3],landmark[5],landmark[7],
                                 landmark[2],landmark[4],landmark[6],landmark[8],
                                    1,             1,          1,          1;

                    apex_dst = apex_norm;

                    for (int i = 0; i < 4; i++)
                    {
                        obj.apex[i] = cv::Point2f(apex_norm(0,i),apex_norm(1,i));
                        obj.pts.push_back(obj.apex[i]);
                    }

                    std::vector<cv::Point2f> tmp(obj.apex, obj.apex + 4);
                    obj.rect = cv::boundingRect(tmp);

                    obj.cls = id;
                    obj.prob = armor_score;

                    proposals.emplace_back(obj);
                }
            }
        }
    }
    std::cout << "rount  " << row_ind << std::endl;

    qsort_descent_inplace(proposals);

    if (proposals.size() >= TOPK)
        proposals.resize(TOPK);
    std::vector<int> picked;
    nms_sorted_bboxes(proposals, picked, NMS_THRESH);
    int count = picked.size();
    objects.resize(count);

    for (int i = 0; i < count; i++)
    {
        objects[i] = proposals[picked[i]];
    }
}


void Inference::run(cv::Mat &img)
{
    // cv::resize(img,img,cv::Size(1280,1024));
    std::vector<ArmorObject> objects;

    std::chrono::steady_clock::time_point time_of_process = std::chrono::steady_clock::now();

    if(!detect(img,objects))
    {
        //std::cout << "detect fail " << std::endl;
        return ;
    }
    else
    {
        //std::cout << "detect success" << std::endl;
    }

    sort(objects.begin(),objects.end(),[](ArmorObject& prev, ArmorObject& next)
    {return prev.area > next.area;});

    for(auto object : objects)
    {
        cv::line(img, object.apex[0], object.apex[1],cv::Scalar(193, 182, 255), 1, 8);
        cv::line(img, object.apex[1], object.apex[2],cv::Scalar(193, 182, 255), 1, 8);
        cv::line(img, object.apex[2], object.apex[3],cv::Scalar(193, 182, 255), 1, 8);
        cv::line(img, object.apex[3], object.apex[0],cv::Scalar(193, 182, 255), 1, 8);
        //std::cout << object.cls  << "      " << object.prob << std::endl;
    }

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast <std::chrono::duration < double>>(t2 - time_of_process);
    float FPS = 1/time_used.count();
    //std::cout << "                                 " << FPS << "                                 " << std::endl;

    //cv::imshow("img",img);
}


