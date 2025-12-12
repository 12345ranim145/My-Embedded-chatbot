from django.urls import path, include
from .views import register, activate, CustomLoginView

urlpatterns = [
    path('register/', register, name='register'),
    path('login/', CustomLoginView.as_view(), name='login'),
    path('activate/<uidb64>/<token>/', activate, name='activate'),
    path('', include('django.contrib.auth.urls')),  # password_reset, etc.
]