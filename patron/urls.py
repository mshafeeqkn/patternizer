from django.urls import path

from patron import views

urlpatterns = [
    path('', views.IndexView.as_view())
]
