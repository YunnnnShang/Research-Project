```sh
# ----- Part 1: Direct Evaluation of the baseline model (Model_R0) -----
# (Assuming Model_R0 is at H:\datasets\irobot\runs\detect\train_finetune_v44\weights\best.pt)

# Evaluate on Level 1
yolo val model="path/to/Model_R0/best.pt" data="path/to/irobot_level1.yaml" name=val_direct_on_L1
# Evaluate on Level 2
yolo val model="path/to/Model_R0/best.pt" data="path/to/irobot_level2.yaml" name=val_direct_on_L2
# Evaluate on Level 3
yolo val model="path/to/Model_R0/best.pt" data="path/to/irobot_level3.yaml" name=val_direct_on_L3


# ----- Part 2: Fine-Tuning and Evaluating -----

# Fine-tune and evaluate for Level 1
yolo train model="path/to/Model_R0/best.pt" data="path/to/irobot_level1.yaml" epochs=50 name=finetune_on_L1 workers=0 batch=4
yolo val model=runs/detect/finetune_on_L1/weights/best.pt data="path/to/irobot_level1.yaml" name=val_finetuned_L1

# Fine-tune and evaluate for Level 2
yolo train model="path/to/Model_R0/best.pt" data="path/to/irobot_level2.yaml" epochs=50 name=finetune_on_L2 workers=0 batch=4
yolo val model=runs/detect/finetune_on_L2/weights/best.pt data="path/to/irobot_level2.yaml" name=val_finetuned_L2

# Fine-tune and evaluate for Level 3
yolo train model="path/to/Model_R0/best.pt" data="path/to/irobot_level3.yaml" epochs=50 name=finetune_on_L3 workers=0 batch=4
yolo val model=runs/detect/finetune_on_L3/weights/best.pt data="path/to/irobot_level3.yaml" name=val_finetuned_L3
```
