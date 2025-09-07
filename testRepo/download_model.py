from huggingface_hub import snapshot_download

local_dir = snapshot_download(
    repo_id="YingyingWang/Qwen3_collaboration_v4_q4_k_m",
    local_dir="models/Qwen3_collaboration_v4_q4_k_m",
    local_dir_use_symlinks=False
)

print("Success!：", local_dir)
