const currentTimeElement = document.getElementById('currentTime');
const stepCountElement = document.getElementById('stepCount');
const connectedDevicesElement = document.getElementById('connectedDevices');
const setTimeForm = document.getElementById('setTimeForm');
const setTimeStatusElement = document.getElementById('setTimeStatus');

// 獲取並更新時間
async function updateTime() {
    try {
        const response = await fetch('/time');
        if (response.ok) {
            const timeString = await response.text();
            currentTimeElement.textContent = `目前 ESP32 時間: ${timeString}`;
        } else {
            currentTimeElement.textContent = '無法獲取時間';
            console.error('Failed to fetch time:', response.status);
        }
    } catch (error) {
        currentTimeElement.textContent = '獲取時間時發生錯誤';
        console.error('Error fetching time:', error);
    }
}

// 獲取並更新步數
async function updateStepCount() {
    try {
        const response = await fetch('/steps');
        if (response.ok) {
            const stepString = await response.text();
            stepCountElement.textContent = `記步數: ${stepString}`;
        } else {
             stepCountElement.textContent = '無法獲取步數';
            console.error('Failed to fetch steps:', response.status);
        }
    } catch (error) {
        stepCountElement.textContent = '獲取步數時發生錯誤';
        console.error('Error fetching steps:', error);
    }
}

// 獲取並更新連接設備列表
async function updateConnectedDevices() {
    try {
        const response = await fetch('/devices');
         if (response.ok) {
            const devicesText = await response.text();
            // 假設回傳是逗號分隔的設備名稱，或者直接是 HTML 格式
            // 這裡簡單假設回傳是純文字，直接顯示
            connectedDevicesElement.textContent = `連接設備: ${devicesText}`;
         } else {
             connectedDevicesElement.textContent = '無法獲取設備列表';
            console.error('Failed to fetch devices:', response.status);
         }
    } catch (error) {
        connectedDevicesElement.textContent = '獲取設備列表時發生錯誤';
        console.error('Error fetching devices:', error);
    }
}

// 處理設定時間表單提交
setTimeForm.addEventListener('submit', async function(event) {
    event.preventDefault(); // 阻止默認的表單提交（頁面重新整理）

    const formData = new FormData(setTimeForm);
    const newTimeValue = formData.get('datetime'); // 獲取 input 的值

    if (!newTimeValue) {
        setTimeStatusElement.textContent = '請選擇時間';
        setTimeStatusElement.style.color = 'orange';
        return;
    }

    // 將數據發送給 ESP32 的 /settime endpoint
    try {
        const response = await fetch('/set_time', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/x-www-form-urlencoded', // 或 application/json 如果你選擇用 JSON 發送
            },
            body: `datetime=${encodeURIComponent(newTimeValue)}`, // URL-encoded 格式
            // body: JSON.stringify({ newTime: newTimeValue }), // JSON 格式範例
        });

        if (response.ok) {
            const resultText = await response.text();
            setTimeStatusElement.textContent = `設定成功: ${resultText}`;
            setTimeStatusElement.style.color = 'green';
            updateTime(); // 設定成功後更新顯示的時間
        } else {
            const errorText = await response.text();
            setTimeStatusElement.textContent = `設定失敗: ${response.status} - ${errorText}`;
            setTimeStatusElement.style.color = 'red';
            console.error('Failed to set time:', response.status, errorText);
        }
    } catch (error) {
        setTimeStatusElement.textContent = '設定時間時發生錯誤';
        setTimeStatusElement.style.color = 'red';
        console.error('Error setting time:', error);
    }
});

// 頁面載入完成後執行
document.addEventListener('DOMContentLoaded', () => {
    updateTime();
    updateStepCount();
    updateConnectedDevices();

    // 每隔一段時間更新數據 (例如 5 秒)
    setInterval(updateTime, 5000);
    setInterval(updateStepCount, 5000);
    setInterval(updateConnectedDevices, 10000); // 設備列表可以更新慢一點
});