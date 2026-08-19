# Git / GitHub 使用指南

远程仓库：`https://github.com/6sycf/UE5_SoftBody.git`

## ⚠️ 重要：push / pull 前必须先开 VPN

git 已配置代理 `http://127.0.0.1:7890`（Clash）。

- **VPN 开着** → `git push` / `git pull` 正常
- **VPN 关着** → 会失败（代理端口不通）

> 如果你的 VPN 端口不是 7890，改一下：
> ```bash
> git config http.proxy http://127.0.0.1:你的端口
> ```
> 取消代理：
> ```bash
> git config --unset http.proxy
> ```

## 日常流程（就三条命令）

```bash
git pull                          # 开工前拉取最新
git add . && git commit -m "改了什么"   # 提交
git push                          # 上传（记得开 VPN）
```

## 回退版本

```bash
git log --oneline                 # 查看提交历史
git status                        # 查看当前改了什么
git diff                          # 查看具体改动内容
git revert <提交号>               # 撤销某次提交（安全，保留历史）
git reset --hard <提交号>         # 硬回到某版本（丢弃之后的所有提交，慎用）
```

## 版本管理范围

**纳入版本管理（会备份、可回退）：**

- `Source/` —— C++ 源码
- `Plugins/` —— 插件源码与着色器
- `Config/` —— UE 配置
- `Content/` —— 所有资产（模型 / 材质 / 蓝图 / 关卡）
- `ThirdParty/` —— 第三方库（OpenHaptics SDK 属商业版权物，**不纳入**，见 `.gitignore`）
- `*.uproject`

**不纳入（引擎自动生成，无需备份）：**

- `Binaries/`、`Intermediate/`、`Saved/`、`DerivedDataCache/`、`.vs/` 等

## 常见问题

| 报错 | 原因 | 处理 |
|------|------|------|
| `502` / `Connection was reset` | FastGithub 在跑，大文件上传被它中断 | 关闭 FastGithub，改用 VPN |
| `408` / 超时 | 上传太慢，服务器等不及 | 检查 VPN 节点 / 带宽，或换节点 |
| 忘开 VPN 就 push | 代理端口不通 | 开 VPN 后重跑 `git push` 即可 |
