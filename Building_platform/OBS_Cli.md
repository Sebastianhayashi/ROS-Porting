# 从 0 开始使用 OBS

> 本文中的 `你需要` 指的是必要行为，`如果你希望` 指的是相关但非必要的 obs 操作。

## 环境及前置准备

该文档中使用的系统为：fedora42

你**需要**做的：

```
sudo dnf install osc obs -y
```

## 登陆 obs

你**需要**登陆 obs：

```
# 这里使用自建 obs 做为案例，请修改成实际链接
osc -A https://build.tarsier-infra.isrc.ac.cn ls home:Sebastianhayashi:ROS-Jazzy
```

你会得到：

```
orchid@bogon:~$ osc -A https://build.tarsier-infra.isrc.ac.cn ls home:Sebastianhayashi:ROS-Jazzy

Your user account / password are not configured yet.
You will be asked for them below, and they will be stored in
/home/orchid/.config/osc/oscrc for future use.

Creating osc configuration file /home/orchid/.config/osc/oscrc ...
Username [build.tarsier-infra.isrc.ac.cn]: Sebastianhayashi
Password [Sebastianhayashi@build.tarsier-infra.isrc.ac.cn]:
To use keyrings please install python3-keyring.

NUM NAME              DESCRIPTION
1   Transient         Do not store the password and always ask for it [secure, in-memory]
2   Obfuscated config Store the password in obfuscated form in the osc config file [insecure, persistent]
3   Config            Store the password in plain text in the osc config file [insecure, persistent]
Select credentials manager [default=1]: 3

```


> 选项 3 会将属于的账号密码写入到：`~/.config/osc/oscrc`，如果你希望临时的使用 obs 而不是长久使用，请根据实际情况选择。

如果你希望看到你项目的元数据：

```
osc -A https://build.tarsier-infra.isrc.ac.cn meta prj home:Sebastianhayashi:ROS-Jazzy
```

如果你希望你的仓库要加入到当前的源的话，你可以：

```
sudo dnf config-manager --add-repo ' .repo URL'
sudo dnf makecache
```

## checkout 仓库

你**需要**将你的仓库 checkout 到本地，你可以通过网页端快速的创建，这里仅解释如何 checkout。

```
osc -A https://build.tarsier-infra.isrc.ac.cn co home:Sebastianhayashi:ROS-Jazzy
```

> co 是 osc 中的 checkout 的缩写，仓库 checkout 到本地后就能看到。
