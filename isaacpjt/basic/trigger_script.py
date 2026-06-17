import sys
import time
from pxr import Usd, UsdGeom, Gf, Vt, UsdUtils
import omni.physxdemos as demo

def main():
    # sys.argv가 없는 환경(임베디드 Python 등)에서 오류 방지
    if not hasattr(sys, 'argv'):
        sys.argv = ['']

    # 전달된 인자 전체를 출력하여 확인
    print("=" * 50)
    print(f"[Trigger] 전체 인자 개수: {len(sys.argv)}")
    for i, arg in enumerate(sys.argv):
        print(f"[Trigger] sys.argv[{i}] = {arg}")
    print("=" * 50)

    # Isaac Sim이 트리거 이벤트 발생 시 6개 인자를 전달함
    if len(sys.argv) == 6:
        stageId = int(sys.argv[1])       # 현재 USD Stage의 고유 ID
        triggerPath = sys.argv[2]         # 트리거 영역의 Prim 경로 (예: /World/boxTrigger)
        otherPath = sys.argv[3]           # 트리거에 진입/이탈한 물체의 Prim 경로 (예: /World/boxActor)
        eventName = sys.argv[4]           # 이벤트 종류: "EnterEvent" 또는 "LeaveEvent"
        scriptFileName = sys.argv[5]      # 실행된 스크립트 파일의 경로

        print(f"[Trigger] stageId:        {stageId}")
        print(f"[Trigger] triggerPath:     {triggerPath}")
        print(f"[Trigger] otherPath:       {otherPath}")
        print(f"[Trigger] eventName:       {eventName}")
        print(f"[Trigger] scriptFileName:  {scriptFileName}")

        # stageId로 현재 열려 있는 USD Stage를 가져옴
        cache = UsdUtils.StageCache.Get()
        stage = cache.Find(Usd.StageCache.Id.FromLongInt(stageId))

        if stage:
            # 트리거에 진입/이탈한 물체의 Prim을 가져옴
            otherPrim = stage.GetPrimAtPath(otherPath)
            # Mesh 형태로 접근하여 색상 변경이 가능하도록 함
            usdGeom = UsdGeom.Mesh.Get(stage, otherPrim.GetPath())

            if eventName == "LeaveEvent":
                # 이탈 시 → 원래 색상(파란색)으로 복귀
                color = Vt.Vec3fArray([demo.get_primary_color()])
                usdGeom.GetDisplayColorAttr().Set(color)
                print("[Trigger] → 이탈: 파란색 복귀")
            else:
                # 진입 시 → 초록색으로 변경
                color = Vt.Vec3fArray([demo.get_hit_color()])
                usdGeom.GetDisplayColorAttr().Set(color)
                print("[Trigger] → 진입: 초록색 변경")

    # 콘솔 출력을 읽을 시간 확보 (확인 후 제거)
    time.sleep(2)
    pass

# 스크립트가 로드되면 즉시 main() 실행
main()