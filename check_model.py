import sys, torch
sys.path.insert(0, r'C:\Users\888y9\Desktop\rsi_printing\Vision\artec_ffs\ffs_core')
m = torch.load(r'C:\Users\888y9\Desktop\Repo\Fast-FoundationStereo\weights\23-36-37\model_best_bp2_serialize.pth', map_location='cpu', weights_only=False)
print('max_disp:', m.args.max_disp)
print('args:', vars(m.args))
