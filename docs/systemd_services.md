# How to use Linux Systemd  
## Good Starting Guide
https://learn.sparkfun.com/tutorials/how-to-run-a-raspberry-pi-program-on-startup/all

## ChatGPT Guide
To create a systemd service on Linux that launches a script on boot and automatically restarts if it crashes, you'll need to follow these steps:

1. **Create the script** you want to run. Make sure it is executable.

2. **Create a systemd service file** for the script.

### Step 1: Create Your Script

First, let's assume your script is named `myscript.sh` and is located in `/usr/local/bin/`. Make sure your script is executable:

```bash
sudo chmod +x /usr/local/bin/myscript.sh
```

### Step 2: Create the systemd Service File

Create a new systemd service file. We'll name it `myscript.service`. Use a text editor like `nano` or `vim`:

```bash
sudo nano /etc/systemd/system/myscript.service
```

### Step 3: Configure the Service File

Add the following content to the `myscript.service` file:

```ini
[Unit]
Description=My Script Service
After=network.target

[Service]
Type=simple
ExecStart=/usr/local/bin/myscript.sh
Restart=always
RestartSec=5
User=your-username
Environment=DISPLAY=:0

[Install]
WantedBy=multi-user.target
```

#### Explanation:

- **[Unit]**: Describes the service.
  - `Description`: A short description of the service.
  - `After`: Ensures that the service starts only after the network is up.

- **[Service]**: The actual configuration of the service.
  - `Type=simple`: Indicates that the service will be considered running after the `ExecStart` command is executed.
  - `ExecStart`: Specifies the full path to your script.
  - `Restart=always`: Ensures the service restarts automatically if it crashes or is stopped.
  - `RestartSec=5`: Specifies a delay of 5 seconds before restarting the service after a crash.
  - `User`: Specifies the user under which the script should run (replace `your-username` with the actual username).
  - `Environment`: Sets environment variables, such as `DISPLAY`, if your script requires it.

- **[Install]**: Specifies when the service should be started.
  - `WantedBy=multi-user.target`: Makes the service start automatically during the boot process in the multi-user mode.

### Step 4: Enable and Start the Service

After creating the service file, you need to enable it to start on boot and then start it:

```bash
sudo systemctl enable myscript.service
sudo systemctl start myscript.service
```

### Step 5: Check the Service Status

You can check the status of your service using:

```bash
sudo systemctl status myscript.service
```

This command will show whether your service is running, and it will also provide any logs that can help diagnose issues if the service fails to start or crashes.

That's it! Your script should now run at boot and automatically restart if it crashes.

## Log files with Systemd
When you set up a systemd service to run a Python file, the output of any `print` statements (which typically goes to `stdout` or `stderr`) is captured by the systemd service manager. By default, this output is logged to the systemd journal.

Here's how the outputs are handled:

1. **`stdout` and `stderr`**: By default, both `stdout` and `stderr` from the script are sent to the journal. You can view them using the `journalctl` command.

2. **Viewing Logs**:
   - To view the logs for a specific service, you can use:
     ```
     journalctl -u your-service-name.service
     ```
   - This command will show all the output from your service, including any `print` statements.

3. **Customizing Log Output**:
   - You can also customize where `stdout` and `stderr` go by modifying the service file. For example:
     ```ini
     [Service]
     ExecStart=/usr/bin/python3 /path/to/your_script.py
     StandardOutput=journal
     StandardError=journal
     ```
   - You can redirect these outputs to files by setting `StandardOutput=file:/path/to/output.log` and `StandardError=file:/path/to/error.log`.

4. **Persistent Logs**:
   - If you want to ensure that your logs are kept even after a reboot or for longer periods, you can configure systemd-journald to store logs persistently. You can do this by editing `/etc/systemd/journald.conf` and setting `Storage=persistent`.

By default, without any customization, all your `print` statements and any other outputs will be stored in the systemd journal, accessible via `journalctl`.
