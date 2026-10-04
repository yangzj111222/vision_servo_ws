"""Labelme JSON → YOLO Pose txt 转换脚本.

输入: images/train/*.json (Labelme 格式)
输出: labels/train/*.txt (YOLO pose 格式)

标签约定:
  - bounding box: "block"
  - 关键点: "p0", "p1", "p2", "p3"
"""
import json
import os
import glob


def convert_one(json_path, output_dir, class_id=0):
    with open(json_path) as f:
        data = json.load(f)

    img_w = data['imageWidth']
    img_h = data['imageHeight']

    bbox = None
    keypoints = [None] * 4  # p0, p1, p2, p3

    for shape in data['shapes']:
        label = shape['label']
        points = shape['points']

        if label == 'block':
            # 兼容 Rectangle（2个点）和 Polygon（4个点）
            xs = [p[0] for p in points]
            ys = [p[1] for p in points]
            x1 = min(xs)
            y1 = min(ys)
            x2 = max(xs)
            y2 = max(ys)
            bbox = (x1, y1, x2, y2)
        elif label.startswith('p') and len(label) == 2:
            idx = int(label[1])
            if 0 <= idx < 4:
                keypoints[idx] = points[0]

    # 检查完整性
    if bbox is None:
        print(f"[跳过] {json_path}: 缺少 bbox")
        return False
    if any(kp is None for kp in keypoints):
        missing = [i for i, kp in enumerate(keypoints) if kp is None]
        print(f"[跳过] {json_path}: 缺少关键点 {missing}")
        return False

    # 归一化 bbox
    x1, y1, x2, y2 = bbox
    cx = (x1 + x2) / 2 / img_w
    cy = (y1 + y2) / 2 / img_h
    w = (x2 - x1) / img_w
    h = (y2 - y1) / img_h

    # 归一化关键点 (x, y, v)
    kp_strs = []
    for kp in keypoints:
        kx = kp[0] / img_w
        ky = kp[1] / img_h
        kp_strs.append(f"{kx:.6f} {ky:.6f} 2")  # v=2 可见

    line = f"{class_id} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f} " + " ".join(kp_strs)

    base = os.path.splitext(os.path.basename(json_path))[0]
    out_path = os.path.join(output_dir, base + ".txt")
    with open(out_path, 'w') as f:
        f.write(line)

    return True


def convert_split(split_name):
    base_dir = os.path.expanduser(f"~/datasets/cube_pose")
    json_dir = os.path.join(base_dir, "images", split_name)
    out_dir = os.path.join(base_dir, "labels", split_name)
    os.makedirs(out_dir, exist_ok=True)

    json_files = sorted(glob.glob(os.path.join(json_dir, "*.json")))
    print(f"[{split_name}] 找到 {len(json_files)} 个 JSON 文件")

    ok = 0
    fail = 0
    for jp in json_files:
        if convert_one(jp, out_dir):
            ok += 1
        else:
            fail += 1

    print(f"[{split_name}] 成功 {ok}, 失败 {fail}")
    return ok, fail


if __name__ == "__main__":
    convert_split("train")
    convert_split("val")
    print("转换完成")