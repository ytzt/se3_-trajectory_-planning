template<int Dim>
LinearBezierBlendInterpolation<Dim>::LinearBezierBlendInterpolation(
    const Points& viaPoints, double delta, double beta_bar)
    : via_points_(viaPoints), delta_(delta), beta_bar_(beta_bar), total_length_(0.0) {
    
    if (viaPoints.size() < 2) {
        throw std::invalid_argument("At least two via points are required");
    }
    computeOffsetPoints();
    constructSegments();
    computeUhat();
}

template<int Dim>
void LinearBezierBlendInterpolation<Dim>::computeOffsetPoints() {
    int n = via_points_.size();
    q_prime_.resize(n);
    q_dprime_.resize(n);

    // 第一个点：q''_0 = q_0
    q_dprime_[0] = via_points_[0];
    
    // 最后一个点：q'_n = q_n
    q_prime_[n-1] = via_points_[n-1];
    
    // 中间点
    for (int k = 1; k < n-1; k++) {
        // 计算方向向量
        Point dir_prev = (via_points_[k] - via_points_[k-1]).normalized();
        Point dir_next = (via_points_[k+1] - via_points_[k]).normalized();
        
        // 计算q'_k和q''_k
        q_prime_[k] = via_points_[k] - delta_ * dir_prev;
        q_dprime_[k] = via_points_[k] + delta_ * dir_next;
    }
}

template<int Dim>
void LinearBezierBlendInterpolation<Dim>::constructSegments() {
    int n = via_points_.size();
    
    // 第一段：线性段 q''_0 -> q'_1
    segments_.push_back({
        LINEAR,
        q_dprime_[0],   // p0 = q''_0
        q_prime_[1],    // p1 = q'_1
        BezierSegment(), // 空贝塞尔对象
        (q_prime_[1] - q_dprime_[0]).norm()  // λ = |p1 - p0|
    });
    
    // 中间段：交替的贝塞尔段和线性段
    for (int k = 1; k < n-1; k++) {
        // 贝塞尔段：q'_k -> q''_k
        Segment bezier_seg;
        bezier_seg.type = BEZIER;
        bezier_seg.p0 = q_prime_[k];
        bezier_seg.p1 = q_dprime_[k];
        
        // 计算五次贝塞尔曲线的控制点
        typename BezierSegment::Points control_points;
        computeQuinticBezierControlPoints(k, control_points);
        
        // 创建贝塞尔段对象
        bezier_seg.bezier = BezierSegment(control_points);
        
        // λ = 5 * |p1 - p0|
        double seg_length = 0.0;
        for (int i = 0; i < 5; ++i) {
            seg_length += (control_points[i+1] - control_points[i]).norm();
        }
        bezier_seg.lambda = seg_length;
        // bezier_seg.lambda = 5.0 * (control_points[1] - control_points[0]).norm();
        
        segments_.push_back(bezier_seg);
        
        // 线性段：q''_k -> q'_{k+1} (如果不是最后一个)
        if (k < n-2) {
            segments_.push_back({
                LINEAR,
                q_dprime_[k],       // p0 = q''_k
                q_prime_[k+1],      // p1 = q'_{k+1}
                BezierSegment(),    // 空贝塞尔对象
                (q_prime_[k+1] - q_dprime_[k]).norm()  // λ = |p1 - p0|
            });
        }
    }
    
    // 最后一段：线性段 q''_{n-2} -> q'_{n-1} (即q_n)
    segments_.push_back({
        LINEAR,
        q_dprime_[n-2],  // p0 = q''_{n-2}
        q_prime_[n-1],   // p1 = q'_{n-1} = q_n
        BezierSegment(), // 空贝塞尔对象
        (q_prime_[n-1] - q_dprime_[n-2]).norm()  // λ = |p1 - p0|
    });
}

template<int Dim>
void LinearBezierBlendInterpolation<Dim>::computeQuinticBezierControlPoints(
    int k, typename BezierSegment::Points& control_points) {
    
    // 确保有6个控制点
    control_points.resize(6);
    
    // 端点
    Point p0 = q_prime_[k];
    Point p5 = q_dprime_[k];
    
    // 计算切线向量（单位向量）
    Point t0 = (via_points_[k] - q_prime_[k]).normalized();
    Point t5 = (q_dprime_[k] - via_points_[k]).normalized();
    
    double alpha_k = 0.0;
    
    if (beta_bar_ < 1e-6) {
        // β_bar为0时使用公式(8.66)计算α_k
        double a = 256.0 - 49.0 * (t0 + t5).squaredNorm();
        double b = 420.0 * (p5 - p0).dot(t0 + t5);
        double c = -900.0 * (p5 - p0).squaredNorm();
        
        // 解二次方程，取最大正根
        double discriminant = b*b - 4*a*c;
        if (discriminant < 0) {
            throw std::runtime_error("Negative discriminant in alpha_k calculation");
        }
        
        alpha_k = (-b + std::sqrt(discriminant)) / (2*a);
        if (alpha_k <= 0) {
            alpha_k = (-b - std::sqrt(discriminant)) / (2*a);
            if (alpha_k <= 0) {
                throw std::runtime_error("No positive solution for alpha_k");
            }
        }
    } else {
        // 使用原始方法计算α_k（基于四次方程）
        // 这里简化处理，实际应用中应根据原始方法实现
        alpha_k = 1.0; // 简化处理，实际应计算
    }
    
    // 计算控制点（根据公式）
    control_points[0] = p0;
    control_points[1] = p0 + (alpha_k / 5.0) * t0;
    control_points[2] = 2*control_points[1] - p0;
    control_points[5] = p5;
    control_points[4] = p5 - (alpha_k / 5.0) * t5;
    control_points[3] = 2*control_points[4] - p5;
}

template<int Dim>
void LinearBezierBlendInterpolation<Dim>::computeUhat() {
    u_hat_.resize(segments_.size() + 1);
    u_hat_[0] = 0.0;
    total_length_ = 0.0;
    
    for (size_t i = 0; i < segments_.size(); i++) {
        total_length_ += segments_[i].lambda;
        u_hat_[i+1] = total_length_;
    }
}

template<int Dim>
std::pair<size_t, double> LinearBezierBlendInterpolation<Dim>::findSegment(double u_hat) const {
    if (u_hat < 0 || u_hat > total_length_) {
        throw std::out_of_range("u_hat is out of range");
    }
    
    // 找到包含u_hat的段
    for (size_t i = 0; i < segments_.size(); i++) {
        if (u_hat >= u_hat_[i] && u_hat <= u_hat_[i+1]) {
            double u = (u_hat - u_hat_[i]) / segments_[i].lambda;
            return {i, u};
        }
    }
    
    // 如果û接近total_length，则返回最后一段的末端
    return {segments_.size() - 1, 1.0};
}

template<int Dim>
typename LinearBezierBlendInterpolation<Dim>::Point 
LinearBezierBlendInterpolation<Dim>::evaluate(double u_hat) const {
    // 找到对应的段
    auto [segment_idx, u] = findSegment(u_hat);
    
    if (segments_[segment_idx].type == LINEAR) {
        // 线性段插值
        return segments_[segment_idx].p0 + u * (segments_[segment_idx].p1 - segments_[segment_idx].p0);
    } else {
        // 贝塞尔段插值
        // return segments_[segment_idx].bezier.evaluate(u);
        return segments_[segment_idx].bezier.single_evaluate(u);
    }
}

template<int Dim>
typename LinearBezierBlendInterpolation<Dim>::Point 
LinearBezierBlendInterpolation<Dim>::derivative(double u_hat, int order) const {
    // 找到对应的段
    auto [segment_idx, u] = findSegment(u_hat);
    
    if (segments_[segment_idx].type == LINEAR) {
        if (order == 1) {
            return (segments_[segment_idx].p1 - segments_[segment_idx].p0).normalized();
        } else {
            return Point::Zero();
        }
    } else {
        // 贝塞尔段导数
        return segments_[segment_idx].bezier.single_derivative(u, order)/segments_[segment_idx].lambda;
    }
}