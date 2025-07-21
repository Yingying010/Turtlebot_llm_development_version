from huggingface_hub import hf_hub_download

model_path = hf_hub_download(
    repo_id="YingyingWang/Qwen3_base_instructions",
    filename="model.safetensors",  # ✅ 正确的模型文件名
    local_dir="models/Qwen3_base_instructions",  # ✅ 下载到本地目录
    local_dir_use_symlinks=False
)

print("✅ 下载完成，路径为：", model_path)
