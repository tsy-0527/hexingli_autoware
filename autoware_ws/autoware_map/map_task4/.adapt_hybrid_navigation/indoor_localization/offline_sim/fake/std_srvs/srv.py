
class SetBool:
    class Request:
        def __init__(s,data=False): s.data=data
    class Response:
        def __init__(s): s.success=False; s.message=''
class Trigger:
    class Request: pass
    class Response:
        def __init__(s): s.success=False; s.message=''
