using UnityEngine;
public class HeadAndAntennaController : MonoBehaviour
{
[Header("머리 설정")]
public Transform head;
public float headTiltAngle = 10f;   // 머리 좌우 까딱 최대 각도
public float headSpeed = 2f;        // 머리 흔들기 속도


[Header("안테나 설정")]
public Transform antennaLeft;
public Transform antennaRight;
public float antennaSwingAngle = 15f;  // 안테나 기본 흔들림 각도
public float antennaSpeed = 3f;        // 안테나 흔들기 속도
public float targetAntennaAngle = 45f; // 음성 감지 시 좌우로 벌어질 각도

[Header("마이크 설정")]
public float sensitivity = 100f;
public float threshold = 0.01f;

private AudioClip micClip;
private string micDevice;
private bool isFrozen = false;
private float headStartY;
private Quaternion leftStartRot, rightStartRot;

void Start()
{
    // 초기값 저장
    headStartY = head.localEulerAngles.y;
    leftStartRot = antennaLeft.localRotation;
    rightStartRot = antennaRight.localRotation;

    // 마이크 시작
    if (Microphone.devices.Length > 0)
    {
        micDevice = Microphone.devices[0];
        micClip = Microphone.Start(micDevice, true, 1, 44100);
    }
    else
    {
        Debug.LogWarning("마이크 장치를 찾을 수 없습니다.");
    }
}

void Update()
{
    if (!isFrozen && micClip != null)
    {
        float level = GetMicLevel();

        // 음성 감지 → 멈추고 특정 포즈 취하기
        if (level > threshold)
        {
            isFrozen = true;

            // 머리: 현재 각도 그대로 유지
            // 안테나: 초기 회전을 기준으로 X축 ±targetAntennaAngle만큼 회전
            antennaLeft.localRotation = leftStartRot * Quaternion.Euler(-targetAntennaAngle, 0, 0);
            antennaRight.localRotation = rightStartRot * Quaternion.Euler(targetAntennaAngle, 0, 0);
            return;
        }
    }

    if (!isFrozen)
    {
        // 머리 흔들기 (Y축 기준 좌우 까딱)
        float headAngle = Mathf.Sin(Time.time * headSpeed) * headTiltAngle;
        head.localEulerAngles = new Vector3(
            head.localEulerAngles.x,
            headStartY + headAngle,
            head.localEulerAngles.z
        );

        // 안테나 흔들기 (초기 각도 기준, X축 반대 방향으로 움직임)
        float antAngle = Mathf.Sin(Time.time * antennaSpeed) * antennaSwingAngle;
        antennaLeft.localRotation = leftStartRot * Quaternion.Euler(-antAngle, 0, 0);
        antennaRight.localRotation = rightStartRot * Quaternion.Euler(antAngle, 0, 0);
    }
}

// 마이크 입력 음량 계산
float GetMicLevel()
{
    float[] samples = new float[128];
    int micPos = Microphone.GetPosition(micDevice) - samples.Length;
    if (micPos < 0) return 0;
    micClip.GetData(samples, micPos);
    float level = 0f;
    foreach (float s in samples) level += Mathf.Abs(s);
    return (level / samples.Length) * sensitivity;
}
