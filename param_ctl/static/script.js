/**
 * Parameter Remote Controller Script
 */

class ParamManager {
    constructor() {
        this.params = {};
        this.initUI();
        this.initEventListeners();
        this.fetchParams();
    }

    initUI() {
        document.title = 'Parameter Remote Controller';
        document.getElementById('page-title').textContent = 'Parameter Remote Controller';
    }

    initEventListeners() {
        document.getElementById('refresh-btn').addEventListener('click', () => {
            this.fetchParams();
        });

        setInterval(() => this.fetchParams(), 5000);
    }

    async fetchParams() {
        try {
            const response = await fetch('/api/params');
            if (!response.ok) {
                throw new Error('Failed to fetch parameters');
            }
            const data = await response.json();
            this.params = data;
            this.renderParams();
        } catch (error) {
            this.showStatus(error.message, false);
        }
    }

    async updateParam(name, value) {
        try {
            const response = await fetch(`/api/params/${name}`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ value: value })
            });

            if (!response.ok) {
                const errorData = await response.json();
                throw new Error(errorData.error || 'Failed to update parameter');
            }

            const data = await response.json();
            this.params[name].value = data.value;
            const message = `Parameter ${name} has been updated to ${data.value}`;
            this.showStatus(message, true);
        } catch (error) {
            this.showStatus(error.message, false);
        }
    }

    renderParams() {
        const container = document.getElementById('params-container');
        if (!this.params || Object.keys(this.params).length === 0) {
            container.innerHTML = '<p>No parameters available</p>';
            return;
        }

        const sortedParams = Object.entries(this.params).sort(([a], [b]) => a.localeCompare(b));
        container.innerHTML = '';

        for (const [name, param] of sortedParams) {
            const paramItem = document.createElement('div');
            paramItem.className = 'param-item';

            const paramHeader = document.createElement('div');
            paramHeader.className = 'param-header';

            const paramName = document.createElement('div');
            paramName.className = 'param-name';
            paramName.textContent = name;

            const paramType = document.createElement('div');
            paramType.className = 'param-type';
            paramType.textContent = `Type: ${param.type}`;

            paramHeader.appendChild(paramName);
            paramHeader.appendChild(paramType);

            const paramDescription = document.createElement('div');
            paramDescription.className = 'param-description';
            paramDescription.textContent = param.description || 'No description';

            const paramValue = document.createElement('div');
            paramValue.className = 'param-value';

            let input;
            const originalValue = param.value;

            if (param.type === 'int' || param.type === 'float') {
                if (param.value_range) {
                    const [min, max] = param.value_range;
                    const slider = document.createElement('input');
                    slider.type = 'range';
                    slider.min = min;
                    slider.max = max;
                    slider.step = param.type === 'int' ? '1' : '0.1';
                    slider.value = param.value;

                    const numberInput = document.createElement('input');
                    numberInput.type = 'number';
                    numberInput.min = min;
                    numberInput.max = max;
                    numberInput.step = param.type === 'int' ? '1' : '0.1';
                    numberInput.value = param.value;

                    slider.addEventListener('input', () => {
                        numberInput.value = slider.value;
                    });

                    slider.addEventListener('change', () => {
                        this.updateParam(name, parseFloat(slider.value));
                    });

                    numberInput.addEventListener('change', () => {
                        slider.value = numberInput.value;
                        this.updateParam(name, parseFloat(numberInput.value));
                    });

                    paramValue.appendChild(slider);
                    paramValue.appendChild(numberInput);
                } else {
                    input = document.createElement('input');
                    input.type = 'number';
                    input.value = param.value;
                    input.addEventListener('change', () => {
                        this.updateParam(name, parseFloat(input.value));
                    });
                    paramValue.appendChild(input);
                }
            } else if (param.type === 'bool') {
                input = document.createElement('input');
                input.type = 'checkbox';
                input.checked = param.value;
                input.addEventListener('change', () => {
                    this.updateParam(name, input.checked);
                });
                paramValue.appendChild(input);
            } else if (param.type === 'str') {
                input = document.createElement('input');
                input.type = 'text';
                input.value = param.value;
                input.addEventListener('change', () => {
                    this.updateParam(name, input.value);
                });
                paramValue.appendChild(input);
            } else {
                const span = document.createElement('span');
                span.textContent = param.value;
                paramValue.appendChild(span);
            }

            paramItem.appendChild(paramHeader);
            paramItem.appendChild(paramDescription);
            paramItem.appendChild(paramValue);
            container.appendChild(paramItem);
        }
    }

    showStatus(message, isSuccess) {
        const status = document.getElementById('status');
        status.textContent = message;
        status.className = `status ${isSuccess ? 'success' : 'error'}`;
        status.style.display = 'block';

        setTimeout(() => {
            status.style.display = 'none';
        }, 3000);
    }
}

// Initialize the application
document.addEventListener('DOMContentLoaded', () => {
    new ParamManager();
});