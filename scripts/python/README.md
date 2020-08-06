# Python scripts to interact with Beam Position Monitor

## Dependencies

* python3
* pyepics

```bash
    pip3 install pyepics --user
```

## Examples

### Read 10 ADC samples from BPM

```bash
    ./bpm_acq.py 'SI-09SAFE:DI-PBPM-1:' 10 'ADC'
```

### Read 10 TbT samples from BPM

```bash
    ./bpm_acq.py 'SI-09SAFE:DI-PBPM-1:' 10 'TbT'
```
