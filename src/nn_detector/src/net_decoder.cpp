#include <detector/net_decoder.h>
#include <utils/common.h>

NetDecoderBase::NetDecoderBase(toml::value &config,const rclcpp::Logger& _logger): logger(_logger) {
    INPUT_W = config.at("INPUT_W").as_integer();
    INPUT_H = config.at("INPUT_H").as_integer();

    NUM_CLASSES = config.at("NUM_CLASSES").as_integer();
    NUM_COLORS = config.at("NUM_COLORS").as_integer();

    BBOX_CONF_THRESH = config.at("BBOX_CONF_THRESH").as_floating();
}

YOLOv5Decoder::YOLOv5Decoder(toml::value &config, const rclcpp::Logger& _logger) : NetDecoderBase(config,_logger) {
    anchors = toml::get<decltype(anchors)>(config.at("anchors"));
}

void YOLOv5Decoder::set_layer_info(int layer_index, const std::vector<size_t> &dimensions) {
    assert((int)layers.size() == layer_index && "set_layer_info should be called in order");
    assert(dimensions.size() == 5 && "model should be 5-dim");
    assert(dimensions[2] == dimensions[3] && "model output should be square");
    int stride = INPUT_H / dimensions[2];

    YOLOv5LayerInfo layer = {layer_index,(int)dimensions[1],(int)dimensions[2],(int)dimensions[3],(int)dimensions[4],stride};

    RCLCPP_INFO(logger,"layer %d: num_anchors=%d, out_h=%d, out_w=%d, num_outputs=%d, stride=%d", layer_index, layer.num_anchors, layer.out_h, layer.out_w, layer.num_outputs, layer.stride);
    assert(this->check_num_outputs(layer.num_outputs) && "num_output check failed");

    layers.push_back(layer);
    return;
}

bool YOLOv5Decoder::check_num_outputs(int num_outputs) { return num_outputs == 1 + 4 + 10 + NUM_CLASSES + NUM_COLORS; }

void YOLOv5Decoder::decode(int layer_index, const float *prob, std::vector<Armor> &objects) {
    assert((int)layers.size() > layer_index && "layer_index out of range");
    auto [_, na, out_h, out_w, no, stride] = layers[layer_index];

    std::vector<float> anchor;
    switch (stride) {
        case 8:
            anchor = anchors[0];
            break;
        case 16:
            anchor = anchors[1];
            break;
        case 32:
            anchor = anchors[2];
            break;
        case 4:
            anchor = anchors[3];
            break;
        default:
            assert(false && "Unknown layer stride");
            break;
    }
    std::vector<float> pred_data_v;
    pred_data_v.resize(no);
    float* pred_data = pred_data_v.data();
    // float pred_data[no];
    // [x, y, w, h, conf, (x,y)*5, hot(class), hot(color)]
    for (int a_id = 0; a_id < na; ++a_id) {
        for (int h_id = 0; h_id < out_h; ++h_id) {
            for (int w_id = 0; w_id < out_w; ++w_id) {
                int data_idx = (a_id * out_h * out_w + h_id * out_w + w_id) * no;
                float obj_conf = sigmoid(prob[data_idx + 4]);
                if (obj_conf > BBOX_CONF_THRESH) {
                    // std::cout << obj_conf << std::endl;
                    sigmoid(prob + data_idx, pred_data, 5);
                    sigmoid(prob + data_idx + 15, pred_data + 15, NUM_CLASSES + NUM_COLORS);
                    memcpy(pred_data + 5, prob + data_idx + 5, sizeof(float) * 10);
                    int cls_id = std::distance(pred_data + 15, std::max_element(pred_data + 15, pred_data + 15 + NUM_CLASSES));
                    int col_id = std::distance(pred_data + 15 + NUM_CLASSES, std::max_element(pred_data + 15 + NUM_CLASSES, pred_data + 15 + NUM_CLASSES + NUM_COLORS));

                    double final_conf = obj_conf * sqrt(pred_data[15 + cls_id] * pred_data[15 + NUM_CLASSES + col_id]);
                    if (final_conf > BBOX_CONF_THRESH) {
                        // std::cout << final_conf << " " << col_id << " "
                        //           << cls_id << std::endl;
                        Armor now;
                        float x = (pred_data[0] * 2.0 - 0.5 + w_id) * stride;
                        float y = (pred_data[1] * 2.0 - 0.5 + h_id) * stride;
                        float w = pow(pred_data[2] * 2, 2) * anchor[a_id * 2];
                        float h = pow(pred_data[3] * 2, 2) * anchor[a_id * 2 + 1];

                        for (int p = 0; p < 5; ++p) {
                            float px = (pred_data[5 + p * 2] * anchor[a_id * 2] + w_id * stride);
                            float py = (pred_data[5 + p * 2 + 1] * anchor[a_id * 2 + 1] + h_id * stride);
                            // px = std::max(std::min(px, (float)(INPUT_W)), 0.f);
                            // py = std::max(std::min(py, (float)(INPUT_H)), 0.f);
                            now.pts[p] = cv::Point2f(px, py);
                            // std::cout << px << " " << py  << " ";
                        }
                        // std::cout << std::endl;

                        float x0 = (x - w * 0.5);
                        float y0 = (y - h * 0.5);
                        float x1 = (x + w * 0.5);
                        float y1 = (y + h * 0.5);

                        // x0 = std::max(std::min(x0, (float)(INPUT_W)), 0.f);
                        // y0 = std::max(std::min(y0, (float)(INPUT_H)), 0.f);
                        // x1 = std::max(std::min(x1, (float)(INPUT_W)), 0.f);
                        // y1 = std::max(std::min(y1, (float)(INPUT_H)), 0.f);

                        now.rect = cv::Rect(x0, y0, x1 - x0, y1 - y0);
                        now.conf = final_conf;
                        now.color = col_id;
                        now.type = cls_id;
                        now.size = 0;
                        objects.push_back(now);
                    }
                }
            }
        }
    }
    return;
}

YOLOv5_1_Decoder::YOLOv5_1_Decoder(toml::value &config, const rclcpp::Logger& _logger) : YOLOv5Decoder(config, _logger) {
    NUM_TSIZES = config.at("NUM_TSIZES").as_integer();
}

bool YOLOv5_1_Decoder::check_num_outputs(int num_outputs) { return num_outputs == 1 + 4 + 10 + NUM_CLASSES + NUM_COLORS + NUM_TSIZES; }

void YOLOv5_1_Decoder::decode(int layer_index, const float *prob, std::vector<Armor> &objects) {
    assert((int)layers.size() > layer_index && "layer_index out of range");
    auto [_, na, out_h, out_w, no, stride] = layers[layer_index];

    std::vector<float> anchor;
    switch (stride) {
        case 8:
            anchor = anchors[0];
            break;
        case 16:
            anchor = anchors[1];
            break;
        case 32:
            anchor = anchors[2];
            break;
        case 4:
            anchor = anchors[3];
            break;
        default:
            assert(false && "Unknown layer stride");
            break;
    }
    std::vector<float> pred_data_v;
    pred_data_v.resize(no);
    float* pred_data = pred_data_v.data();
    // [x, y, w, h, conf, (x,y)*5, hot(class), hot(color), hot(tsize)]
    for (int a_id = 0; a_id < na; ++a_id) {
        for (int h_id = 0; h_id < out_h; ++h_id) {
            for (int w_id = 0; w_id < out_w; ++w_id) {
                int data_idx = (a_id * out_h * out_w + h_id * out_w + w_id) * no;
                float obj_conf = sigmoid(prob[data_idx + 4]);
                if (obj_conf > BBOX_CONF_THRESH) {
                    // std::cout << obj_conf << std::endl;
                    sigmoid(prob + data_idx, pred_data, 5);
                    sigmoid(prob + data_idx + 15, pred_data + 15, NUM_CLASSES + NUM_COLORS + NUM_TSIZES);
                    memcpy(pred_data + 5, prob + data_idx + 5, sizeof(float) * 10);
                    int cls_id = std::distance(pred_data + 15, std::max_element(pred_data + 15, pred_data + 15 + NUM_CLASSES));
                    int col_id = std::distance(pred_data + 15 + NUM_CLASSES, std::max_element(pred_data + 15 + NUM_CLASSES, pred_data + 15 + NUM_CLASSES + NUM_COLORS));
                    int ts_id =
                        std::distance(pred_data + 15 + NUM_CLASSES + NUM_COLORS, std::max_element(pred_data + 15 + NUM_CLASSES + NUM_COLORS, pred_data + 15 + NUM_CLASSES + NUM_COLORS + NUM_TSIZES));

                    double final_conf = obj_conf * pow(pred_data[15 + cls_id] * pred_data[15 + NUM_CLASSES + col_id] * pred_data[15 + NUM_CLASSES + NUM_COLORS + ts_id], 1.0 / 3.0);
                    if (final_conf > BBOX_CONF_THRESH) {
                        // std::cout << final_conf << " " << col_id << " "
                        //           << cls_id << std::endl;
                        Armor now;
                        float x = (pred_data[0] * 2.0 - 0.5 + w_id) * stride;
                        float y = (pred_data[1] * 2.0 - 0.5 + h_id) * stride;
                        float w = pow(pred_data[2] * 2, 2) * anchor[a_id * 2];
                        float h = pow(pred_data[3] * 2, 2) * anchor[a_id * 2 + 1];

                        for (int p = 0; p < 5; ++p) {
                            float px = (pred_data[5 + p * 2] * anchor[a_id * 2] + w_id * stride);
                            float py = (pred_data[5 + p * 2 + 1] * anchor[a_id * 2 + 1] + h_id * stride);
                            // px = std::max(std::min(px, (float)(INPUT_W)), 0.f);
                            // py = std::max(std::min(py, (float)(INPUT_H)), 0.f);
                            now.pts[p] = cv::Point2f(px, py);
                            // std::cout << px << " " << py  << " ";
                        }
                        // std::cout << std::endl;

                        float x0 = (x - w * 0.5);
                        float y0 = (y - h * 0.5);
                        float x1 = (x + w * 0.5);
                        float y1 = (y + h * 0.5);

                        // x0 = std::max(std::min(x0, (float)(INPUT_W)), 0.f);
                        // y0 = std::max(std::min(y0, (float)(INPUT_H)), 0.f);
                        // x1 = std::max(std::min(x1, (float)(INPUT_W)), 0.f);
                        // y1 = std::max(std::min(y1, (float)(INPUT_H)), 0.f);

                        now.rect = cv::Rect(x0, y0, x1 - x0, y1 - y0);
                        now.conf = final_conf;
                        now.color = col_id;
                        now.type = cls_id;
                        now.size = ts_id;
                        objects.push_back(now);
                    }
                }
            }
        }
    }
    return;
}

DETRDecoder::DETRDecoder(toml::value &config, const rclcpp::Logger &_logger)
    : NetDecoderBase(config, _logger) {
    try {
        min_class_score = static_cast<float>(config.at("DETR_MIN_CLASS_SCORE").as_floating());
    } catch (...) {
        min_class_score = 0.0f;
    }
    try {
        min_color_score = static_cast<float>(config.at("DETR_MIN_COLOR_SCORE").as_floating());
    } catch (...) {
        min_color_score = 0.0f;
    }
    try {
        class_margin = static_cast<float>(config.at("DETR_CLASS_MARGIN").as_floating());
    } catch (...) {
        class_margin = 0.0f;
    }
    try {
        color_margin = static_cast<float>(config.at("DETR_COLOR_MARGIN").as_floating());
    } catch (...) {
        color_margin = 0.0f;
    }
}

void DETRDecoder::set_layer_info(int layer_index, const std::vector<size_t> &dims) {
    assert((int)layers.size() == layer_index && "set_layer_info should be called in order");
    assert(dims.size() == 3 && "DETR model output should be 3-dim [batch, queries, outputs]");

    DETRLayerInfo layer = {
        layer_index,
        static_cast<int>(dims[1]),  // num_queries
        static_cast<int>(dims[2]),  // num_outputs per query
    };
    RCLCPP_INFO(logger, "[DETR] layer %d: num_queries=%d, num_outputs=%d",
                layer_index, layer.num_queries, layer.num_outputs);
    assert(check_num_outputs(layer.num_outputs) && "DETR num_output check failed");
    layers.push_back(layer);
}

bool DETRDecoder::check_num_outputs(int num_outputs) {
    return num_outputs == 4 + NUM_CLASSES + NUM_COLORS;
}

void DETRDecoder::decode(int layer_index, const float *prob, std::vector<Armor> &objects) {
    assert((int)layers.size() > layer_index && "layer_index out of range");
    const auto &layer = layers[layer_index];
    const int no = layer.num_outputs;
    const int nq = layer.num_queries;

    for (int q = 0; q < nq; ++q) {
        const float *row = prob + q * no;
        // bbox: cx, cy, w, h (normalized 0-1 after sigmoid)
        float cx = row[0] * INPUT_W;
        float cy = row[1] * INPUT_H;
        float bw  = row[2] * INPUT_W;
        float bh  = row[3] * INPUT_H;

        // class and color scores (already sigmoid)
        const float *cls_scores = row + 4;
        const float *col_scores = row + 4 + NUM_CLASSES;

        int cls_id = 0;
        int cls_second_id = 0;
        float cls_best = cls_scores[0];
        float cls_second = -1.0f;
        for (int idx = 1; idx < NUM_CLASSES; ++idx) {
            if (cls_scores[idx] > cls_best) {
                cls_second = cls_best;
                cls_second_id = cls_id;
                cls_best = cls_scores[idx];
                cls_id = idx;
            } else if (cls_scores[idx] > cls_second) {
                cls_second = cls_scores[idx];
                cls_second_id = idx;
            }
        }

        int col_id = 0;
        int col_second_id = 0;
        float col_best = col_scores[0];
        float col_second = -1.0f;
        for (int idx = 1; idx < NUM_COLORS; ++idx) {
            if (col_scores[idx] > col_best) {
                col_second = col_best;
                col_second_id = col_id;
                col_best = col_scores[idx];
                col_id = idx;
            } else if (col_scores[idx] > col_second) {
                col_second = col_scores[idx];
                col_second_id = idx;
            }
        }

        if (cls_best < min_class_score)
            continue;
        if (col_best < min_color_score)
            continue;
        if (cls_second_id != cls_id && cls_best - cls_second < class_margin)
            continue;
        if (col_second_id != col_id && col_best - col_second < color_margin)
            continue;

        // Use geometric mean to avoid over-penalizing class/color joint confidence.
        float final_conf = std::sqrt(std::max(0.0f, cls_best * col_best));
        if (final_conf <= BBOX_CONF_THRESH)
            continue;

        // Matcher only consumes type [0..5]. Skip other classes (0/Bs/Bb-like) here.
        if (cls_id > 5)
            continue;

        // Matcher only consumes BLUE/RED colors. Skip N/P-like classes here.
        if (col_id > 1)
            continue;

        Armor now;
        float x0 = cx - bw * 0.5f;
        float y0 = cy - bh * 0.5f;
        float x1 = cx + bw * 0.5f;
        float y1 = cy + bh * 0.5f;
        x0 = std::max(0.0f, std::min(x0, static_cast<float>(INPUT_W - 1)));
        y0 = std::max(0.0f, std::min(y0, static_cast<float>(INPUT_H - 1)));
        x1 = std::max(0.0f, std::min(x1, static_cast<float>(INPUT_W - 1)));
        y1 = std::max(0.0f, std::min(y1, static_cast<float>(INPUT_H - 1)));
        if (x1 <= x0 || y1 <= y0)
            continue;
        now.rect  = cv::Rect(x0, y0, x1 - x0, y1 - y0);
        now.conf  = final_conf;
        now.color = col_id;
        now.type  = cls_id;
        now.size  = 0;
        // derive 4 corner pts + center (pts[4])
        now.pts[0] = cv::Point2f(x0, y0);
        now.pts[1] = cv::Point2f(x1, y0);
        now.pts[2] = cv::Point2f(x1, y1);
        now.pts[3] = cv::Point2f(x0, y1);
        now.pts[4] = cv::Point2f(cx, cy);
        objects.push_back(now);
    }
}

YOLOv8Decoder::YOLOv8Decoder(toml::value &config, const rclcpp::Logger& _logger) : NetDecoderBase(config, _logger) {
    NUM_KPTS = config.at("NUM_KPTS").as_integer();
    NUM_TSIZES = config.at("NUM_TSIZES").as_integer();
}

void YOLOv8Decoder::set_layer_info(int layer_index, const std::vector<size_t> &dims) {
    assert((int)layers.size() == layer_index && "set_layer_info should be called in order");
    assert(dims.size() == 3 && "model should be 3-dim");
    assert(dims[1] == 8400);

    // YOLOv8LayerInfo layer{
    //     .index = layer_index,
    //     .num_outputs = static_cast<int>(dims[2]),
    // };
    YOLOv8LayerInfo layer = {layer_index, (int)dims[2], 0};

    RCLCPP_INFO(logger,"layer %d: num_outputs=%d", layer_index, layer.num_outputs);
    assert(this->check_num_outputs(layer.num_outputs) && "num_output check failed");

    layers.push_back(layer);
    return;
}

bool YOLOv8Decoder::check_num_outputs(int num_outputs) { return num_outputs == NUM_KPTS + NUM_CLASSES + NUM_COLORS + NUM_TSIZES; }

void YOLOv8Decoder::decode(int layer_index, const float *prob, std::vector<Armor> &objects) {
    assert((int)layers.size() > layer_index && "layer_index out of range");
    int no = layers[layer_index].num_outputs;
    std::vector<float> pred_data_v;
    pred_data_v.resize(no);
    float* pred_data = pred_data_v.data();
    // [kpts(8), hot(classes)(8), hot(tsizes)(2), hot(colors)(4)]
    for (int idx = 0; idx < 8400; ++idx) {
        float rough_conf = *std::max_element(&prob[idx * no + NUM_KPTS], &prob[idx * no + no]);

        if (rough_conf > BBOX_CONF_THRESH) {
            std::memcpy(pred_data, &prob[idx * no], no * sizeof(float));
            int cls_id = std::distance(pred_data + NUM_KPTS, std::max_element(pred_data + NUM_KPTS, pred_data + NUM_KPTS + NUM_CLASSES));
            int ts_id = std::distance(pred_data + NUM_KPTS + NUM_CLASSES, std::max_element(pred_data + NUM_KPTS + NUM_CLASSES, pred_data + NUM_KPTS + NUM_CLASSES + NUM_TSIZES));
            int col_id = std::distance(pred_data + NUM_KPTS + NUM_CLASSES + NUM_TSIZES,
                                       std::max_element(pred_data + NUM_KPTS + NUM_CLASSES + NUM_TSIZES, pred_data + NUM_KPTS + NUM_CLASSES + NUM_TSIZES + NUM_COLORS));

            double final_conf = std::min({pred_data[NUM_KPTS + NUM_CLASSES + NUM_TSIZES + col_id], pred_data[NUM_KPTS + cls_id]});
            if (final_conf > BBOX_CONF_THRESH) {
                // std::cout << final_conf << " " << col_id << " "
                //           << cls_id << std::endl;
                Armor now;

                for (int p = 0; p < (NUM_KPTS / 2); ++p) {
                    // float px = std::max(std::min(pred_data[p * 2], (float)(INPUT_W)), 0.f);
                    // float py = std::max(std::min(pred_data[p * 2 + 1], (float)(INPUT_H)), 0.f);
                    float px = pred_data[p * 2];
                    float py = pred_data[p * 2 + 1];
                    now.pts[p] = cv::Point2f(px, py);
                }

                now.rect = cv::Rect(now.pts[0], now.pts[2]);
                now.conf = final_conf;
                now.color = col_id;
                now.type = cls_id;
                now.size = ts_id;
                objects.push_back(now);
            }
        }
    }
}