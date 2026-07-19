"""X42S 原点回零命令."""

import logging

from ..configs import Code, HomingMode, Protocol, StatusCode, StoreFlag, SyncFlag
from ..parameters import DeviceParams, HomingParams, HomingStatus
from .base import Command, SimpleCommand

logger = logging.getLogger(__name__)


class SetHomeZero(SimpleCommand):
    """设置单圈回零的零点位置.

    对应命令: 5.4.1 设置单圈回零的零点位置
    发送: 01 93 88 01 6B
    返回: 01 93 02 6B
    """

    _code = Code.SET_HOME_ZERO
    _protocol = Protocol.SET_HOME_ZERO

    def __init__(self, device: DeviceParams, store: bool = True):
        self.store = store
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([
            self.address,
            self._code,
            self._protocol,
            StoreFlag.STORE if self.store else StoreFlag.NO_STORE,
        ])


class Home(SimpleCommand):
    """触发回零.

    对应命令: 5.4.2 触发回零
    发送: 01 9A 00 00 6B
    返回: 01 9A 02 6B (已在零点时可能为 12)
    """

    _code = Code.HOME

    def __init__(
        self,
        device: DeviceParams,
        mode: HomingMode = HomingMode.NEAREST,
        sync_flag: SyncFlag = SyncFlag.IMMEDIATE,
    ):
        self.mode = mode
        self.sync_flag = sync_flag
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code, self.mode, self.sync_flag])

    def _parse_response(self, data: bytes) -> bool:
        # 手册: 02/9F/E2/EE；9F=回零完成，12/22=已在零点/限位不动作
        if data[0] in (
            StatusCode.SUCCESS,
            StatusCode.ACTION_COMPLETE,
            StatusCode.AT_ZERO,
            StatusCode.LIMIT_OR_HOME,
        ):
            return True
        if data[0] == StatusCode.PARAM_ERROR:
            logger.warning("回零命令参数错误(若已在零点可先离开再试)")
            return False
        if data[0] == StatusCode.FORMAT_ERROR:
            logger.warning("回零命令格式错误")
            return False
        return False


class StopHome(SimpleCommand):
    """强制中断并退出回零操作.

    对应命令: 5.4.3 强制中断并退出回零操作
    发送: 01 9C 48 6B
    返回: 01 9C 02 6B
    """

    _code = Code.STOP_HOME
    _protocol = Protocol.STOP_HOME

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code, self._protocol])


class GetHomingStatus(Command[HomingStatus]):
    """读取回零状态标志.

    对应命令: 5.4.4 读取回零状态标志
    发送: 01 3B 6B
    返回: 01 3B 03 6B
    """

    _code = Code.GET_HOME_STATUS
    _response_length = 4

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code])

    def _parse_response(self, data: bytes) -> HomingStatus:
        return HomingStatus.from_byte(data[0])


class GetHomingParams(Command[HomingParams]):
    """读取回零参数.

    对应命令: 5.4.5 读取回零参数
    发送: 01 22 6B
    返回: 01 22 + 15字节数据 + 6B
    """

    _code = Code.GET_HOME_PARAM
    _response_length = 18  # 地址 + 功能码 + 15字节数据 + 校验

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code])

    def _parse_response(self, data: bytes) -> HomingParams:
        return HomingParams.from_bytes(data)


class SetHomingParams(SimpleCommand):
    """修改回零参数.

    对应命令: 5.4.6 修改回零参数
    """

    _code = Code.SET_HOME_PARAM
    _protocol = Protocol.SET_HOME_PARAM

    def __init__(self, device: DeviceParams, params: HomingParams, store: bool = True):
        self.params = params
        self.store = store
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([
            self.address,
            self._code,
            self._protocol,
            StoreFlag.STORE if self.store else StoreFlag.NO_STORE,
        ]) + self.params.bytes
