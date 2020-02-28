function bboxOverlap = getBboxOverlap(bb1, bb2)
    x_left   = max(bb1(1), bb2(1));
    y_top    = max(bb1(2), bb2(2));
    x_right  = min(bb1(1)+bb1(3), bb2(1)+bb2(3));
    y_bottom = min(bb1(2)+bb1(4), bb2(2)+bb2(4));
    
    intersection_area = (x_right - x_left) * (y_bottom - y_top);
    
    bb1_area = bb1(3) * bb1(4);
    bb2_area = bb2(3) * bb2(4);
    
    bboxOverlap = intersection_area / (bb1_area + bb2_area - intersection_area);
end