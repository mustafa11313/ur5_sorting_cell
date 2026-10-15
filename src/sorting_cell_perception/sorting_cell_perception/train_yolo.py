"""Train the part detector on a dataset made by dataset_recorder.

    ros2 run sorting_cell_perception train_yolo --data ~/sorting_dataset/data.yaml \
        --epochs 60 --out src/sorting_cell_perception/models/parts_yolo.pt

Strong colour/brightness augmentation is used on purpose: it is what lets the
model cope with the lighting-change and new-variant scenarios.
"""
import argparse
import json
import os
import shutil


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('--data', required=True, help='data.yaml written by dataset_recorder')
    p.add_argument('--base', default='yolov8n.pt', help='pretrained weights to fine-tune')
    p.add_argument('--epochs', type=int, default=60)
    p.add_argument('--imgsz', type=int, default=640)
    p.add_argument('--batch', type=int, default=16)
    p.add_argument('--device', default='cpu', help="'cpu' or a CUDA index such as '0'")
    p.add_argument('--workers', type=int, default=4)
    p.add_argument('--project', default=os.path.expanduser('~/sorting_runs'))
    p.add_argument('--out', default='parts_yolo.pt', help='where to copy the best weights')
    args = p.parse_args()

    from ultralytics import YOLO

    model = YOLO(args.base)
    model.train(
        data=args.data, epochs=args.epochs, imgsz=args.imgsz, batch=args.batch,
        device=args.device, workers=args.workers, project=args.project, name='parts',
        exist_ok=True, patience=20, seed=0, deterministic=True,
        # augmentation tuned for an overhead camera over a belt
        hsv_h=0.03, hsv_s=0.6, hsv_v=0.5, degrees=180.0, translate=0.1, scale=0.25,
        flipud=0.5, fliplr=0.5, mosaic=1.0, close_mosaic=10,
        plots=True, verbose=False)

    best = os.path.join(args.project, 'parts', 'weights', 'best.pt')
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    shutil.copy(best, args.out)
    metrics = YOLO(args.out).val(data=args.data, device=args.device, imgsz=args.imgsz,
                                 plots=False, verbose=False)
    names = metrics.names
    per_class = {names[int(c)]: {'precision': float(metrics.box.p[i]),
                                 'recall': float(metrics.box.r[i]),
                                 'mAP50': float(metrics.box.ap50[i])}
                 for i, c in enumerate(metrics.box.ap_class_index)}
    summary = {'mAP50': float(metrics.box.map50), 'mAP50_95': float(metrics.box.map),
               'precision': float(metrics.box.mp), 'recall': float(metrics.box.mr),
               'per_class': per_class}
    with open(os.path.splitext(args.out)[0] + '_metrics.json', 'w') as f:
        json.dump(summary, f, indent=2)
    print(json.dumps(summary, indent=2))
    print(f'weights saved to {args.out}')


if __name__ == '__main__':
    main()
