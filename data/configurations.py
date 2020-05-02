class BaseConfig(object):
 '''
 Base config class
 '''
 DEBUG = True
 TESTING = Falseclass ProductionConfig(BaseConfig):
 """
 Production specific config
 """
 DEBUG = Falseclass DevelopmentConfig(BaseConfig):
 """
 Development environment specific configuration
 """
 DEBUG = True
 TESTING = True