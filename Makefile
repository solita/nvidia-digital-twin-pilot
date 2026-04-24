.PHONY: help up down dashboard dashboard-dev logs logs-dashboard status clean restart

COMPOSE := docker compose -f local-compose.yml

help: ## Show available commands
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36mmake %-18s\033[0m %s\n", $$1, $$2}'

up: ## Start core services (redis + warehouse-manager)
	@source .env && export BREV_TAILSCALE_IP && \
		envsubst < fastdds_client.xml > fastdds_client_resolved.xml
	$(COMPOSE) up --build -d

down: ## Stop all services
	$(COMPOSE) --profile dashboard down

dashboard: ## Start dashboard (http://localhost:3000)
	$(COMPOSE) --profile dashboard up --build -d dashboard
	@echo "\n  Dashboard running at: http://localhost:3000\n"

dashboard-dev: ## Start dashboard in Vite dev mode (http://localhost:5173)
	cd dashboard && npm install && npm run dev

logs: ## Tail logs for core services
	$(COMPOSE) logs -f

logs-dashboard: ## Tail dashboard logs
	$(COMPOSE) --profile dashboard logs -f dashboard

status: ## Show running containers
	$(COMPOSE) --profile dashboard ps

restart: ## Restart all running services
	$(COMPOSE) --profile dashboard restart

clean: ## Stop everything and remove volumes
	$(COMPOSE) --profile dashboard down -v
